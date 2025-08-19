#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import threading
import socket
import json
import os
import subprocess
import time
import signal
import shutil
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.parse
from dotenv import load_dotenv
from pathlib import Path

class CameraManager:
    def __init__(self, mediamtx_server_ip='192.168.1.74'):
        self.mediamtx_server_ip = mediamtx_server_ip
        self.camera_stream_process = None
        self.is_streaming = False
        self._stderr_thread = None
        self.fifo_path = '/tmp/pi_patrol_frames'
    
    def start_camera_stream(self, device='/dev/video0', stream_name='pi_patrol'):
        try:
            # Stream to external MediaMTX server via RTSP
            user = os.getenv('MEDIAMTX_USER', '')
            passwd = os.getenv('MEDIAMTX_PASS', '')
            auth_prefix = ''
            if user and passwd:
                auth_prefix = f'{user}:{passwd}@'
            rtsp_url = f'rtsp://{auth_prefix}{self.mediamtx_server_ip}:8554/{stream_name}'
            
            # Optional pre-check: ensure RTSP port reachable
            try:
                with socket.create_connection((self.mediamtx_server_ip, 8554), timeout=2.0):
                    pass
            except Exception as e:
                print(f"Warning: Cannot reach MediaMTX at {self.mediamtx_server_ip}:8554 - {e}")
            
            # Ensure FIFO exists so ffmpeg can open it for reading and block until writer connects
            try:
                if not os.path.exists(self.fifo_path):
                    os.mkfifo(self.fifo_path)
            except FileExistsError:
                pass
            except Exception as e:
                print(f"Failed to create FIFO {self.fifo_path}: {e}")
                return False

            # Build low-latency ffmpeg pipeline reading rawvideo from FIFO
            cmd = [
                'ffmpeg',
                '-hide_banner',
                '-nostats',
                '-loglevel', 'error',
                # Input low-latency flags
                '-fflags', 'nobuffer',
                '-flags', 'low_delay',
                '-use_wallclock_as_timestamps', '1',
                '-probesize', '32',
                '-analyzeduration', '0',
                '-thread_queue_size', '64',
                # Raw video input from FIFO (BGR24 frames from camera_node)
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-video_size', '640x480',
                '-framerate', '15',
                '-i', self.fifo_path,
                # Encode H.264 for RTSP
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-pix_fmt', 'yuv420p',
                '-g', '15',
                '-keyint_min', '15',
                '-bf', '0',
                '-x264-params', 'scenecut=0:rc-lookahead=0',
                '-f', 'rtsp',
                '-rtsp_transport', 'udp',
                rtsp_url
            ]

            self.camera_stream_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=1,
                universal_newlines=True
            )

            # Drain stderr to avoid blocking and provide diagnostics
            def _drain_stderr(proc):
                try:
                    for line in proc.stderr:
                        if not line:
                            break
                        print(f"[ffmpeg] {line.rstrip()}")
                except Exception:
                    pass
            self._stderr_thread = threading.Thread(target=_drain_stderr, args=(self.camera_stream_process,), daemon=True)
            self._stderr_thread.start()

            print(f"Attempting camera stream from FIFO {self.fifo_path} to MediaMTX server at {rtsp_url}")

            # Quick health check: if ffmpeg exits immediately, report failure
            time.sleep(0.7)
            ret = self.camera_stream_process.poll()
            if ret is None:
                self.is_streaming = True
                print(f"Camera stream started (FIFO -> RTSP) to {rtsp_url}")
                return True
            else:
                self.is_streaming = False
                print(f"FFmpeg exited quickly with code {ret}; stream not started")
                self.camera_stream_process = None
                return False
            
        except Exception as e:
            print(f"Failed to start camera stream: {e}")
            self.is_streaming = False
            return False
    
    def stop_camera_stream(self):
        try:
            if self.camera_stream_process:
                self.camera_stream_process.terminate()
                try:
                    self.camera_stream_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.camera_stream_process.kill()
                    self.camera_stream_process.wait(timeout=3)
                self.camera_stream_process = None
            self.is_streaming = False
            print("Camera stream stopped")
            return True
        except Exception as e:
            print(f"Error stopping camera stream: {e}")
            return False
    
    def restart_camera_stream(self, device='/dev/video0', stream_name='pi_patrol'):
        self.stop_camera_stream()
        time.sleep(1)
        return self.start_camera_stream(device, stream_name)
    
    def get_stream_status(self):
        if self.camera_stream_process:
            return_code = self.camera_stream_process.poll()
            if return_code is None:
                return {'active': True, 'status': 'streaming'}
            else:
                self.is_streaming = False
                return {'active': False, 'status': f'stopped (exit code: {return_code})'}
        return {'active': False, 'status': 'not started'}

class ServerNode(Node):
    def __init__(self, mediamtx_server_ip='192.168.1.74'):
        super().__init__('server_node')
        
        # Load environment variables from .env file
        if 'ROS_WORKSPACE' in os.environ:
            workspace_dir = os.environ['ROS_WORKSPACE']
        else:
            # Try to determine workspace from file path
            current_dir = Path(__file__).resolve().parent  # pi_patrol package python dir
            package_dir = current_dir.parent  # pi_patrol package dir
            src_dir = package_dir.parent  # src dir
            workspace_dir = src_dir.parent  # workspace dir
        package_name = 'pi_patrol'
        
        # Build paths dynamically - try multiple locations without hardcoding
        possible_paths = [
            # Current directory
            Path.cwd() / '.env',
            # Source directory (current package)
            Path(os.path.dirname(os.path.dirname(__file__))) / '.env',
            # Package directory
            Path(current_dir.parent) / '.env',
            # Workspace root
            Path(workspace_dir) / '.env',
            # Package directory in workspace
            Path(workspace_dir) / 'src' / package_name / '.env',
            # Home directory
            Path.home() / '.env',
            # ROS_HOME directory if set
            Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / '.env'
        ]
        
        env_loaded = False
        for dotenv_path in possible_paths:
            self.get_logger().info(f'Looking for .env file at: {dotenv_path.absolute()}')
            if dotenv_path.exists():
                load_dotenv(dotenv_path)
                self.get_logger().info(f'Found and loaded .env file at: {dotenv_path.absolute()}')
                env_loaded = True
                break
        
        # Set up ROS2 publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.command_publisher = self.create_publisher(String, '/server/command', 10)
        self.notifications_publisher = self.create_publisher(Bool, '/notifications_enabled', 10)
        
        # Initialize camera manager
        self.camera_manager = CameraManager(mediamtx_server_ip)
        self.streaming_active = False
        
        # Operational state
        self.mode = os.getenv('DEFAULT_MODE', 'disarmed')
        self.notifications_enabled = os.getenv('NOTIFICATIONS_DEFAULT', 'true').lower() == 'true'
        self.recordings_dir = os.environ.get('PI_PATROL_RECORDINGS_DIR', os.path.expanduser("~/tracking_recordings"))

        # Publish initial state
        try:
            init_mode_msg = String()
            init_mode_msg.data = self.mode
            self.robot_mode_publisher.publish(init_mode_msg)
            init_notif_msg = Bool()
            init_notif_msg.data = self.notifications_enabled
            self.notifications_publisher.publish(init_notif_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish initial state: {e}")
        
        # Get port from ROS parameter
        self.port = self.declare_parameter('port', 8080).value
        
        # Get MediaMTX URL
        self.mediamtx_url = f'rtsp://{mediamtx_server_ip}:8554/pi_patrol'
        
        # Start HTTP server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f'Server node started on port {self.port}')
    
    def _get_default_iface(self):
        """Best-effort detection of default network interface (fallback to wlan0)."""
        try:
            with open('/proc/net/route', 'r') as f:
                for line in f.readlines()[1:]:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        iface, dest, flags_hex = parts[0], parts[1], parts[3]
                        if dest == '00000000' and (int(flags_hex, 16) & 0x2):
                            return iface
        except Exception:
            pass
        return os.getenv('WIFI_IFACE', 'wlan0')

    def get_wifi_info(self):
        """Return Wi‑Fi info dict with keys: interface, signal_dbm, link_quality, source; or None."""
        iface = os.getenv('WIFI_IFACE') or self._get_default_iface()
        # Try /proc/net/wireless first
        try:
            with open('/proc/net/wireless', 'r') as f:
                lines = f.readlines()
            selected = None
            for ln in lines[2:]:
                ln = ln.strip()
                if not ln:
                    continue
                ifname = ln.split(':', 1)[0].strip()
                if ifname == iface:
                    selected = ln
                    break
                if selected is None:
                    selected = ln
            if selected:
                ifname = selected.split(':', 1)[0].strip()
                rest = selected.split(':', 1)[1].strip()
                toks = rest.split()
                if len(toks) >= 3:
                    def _to_float(s):
                        try:
                            return float(s.strip().rstrip('.'))
                        except Exception:
                            return None
                    link = _to_float(toks[0])
                    level = _to_float(toks[1])
                    return {'interface': ifname, 'signal_dbm': level, 'link_quality': link, 'source': '/proc/net/wireless'}
        except Exception:
            pass
        # Fallback to `iw`
        try:
            if shutil.which('iw') and iface:
                out = subprocess.check_output(['iw', 'dev', iface, 'link'], text=True, timeout=1.5)
                sig = None
                for line in out.splitlines():
                    if 'signal:' in line:
                        try:
                            sig = float(line.split('signal:')[1].split('dBm')[0].strip())
                        except Exception:
                            sig = None
                        break
                if sig is not None:
                    return {'interface': iface, 'signal_dbm': sig, 'source': 'iw'}
        except Exception:
            pass
        return None

    def get_battery_info(self):
        """Return battery info dict with keys: percent, voltage_v, status, power_supply; or None."""
        base = '/sys/class/power_supply'
        try:
            if os.path.isdir(base):
                candidates = []
                for name in os.listdir(base):
                    ps_path = os.path.join(base, name)
                    tpath = os.path.join(ps_path, 'type')
                    p_type = None
                    if os.path.isfile(tpath):
                        try:
                            with open(tpath, 'r') as f:
                                p_type = f.read().strip()
                        except Exception:
                            p_type = None
                    if (p_type and p_type.lower() == 'battery') or ('bat' in name.lower()):
                        candidates.append((name, ps_path, p_type))
                candidates.sort(key=lambda x: (0 if (x[2] and x[2].lower() == 'battery') else 1))
                for name, ps_path, _ in candidates:
                    percent = None
                    voltage_v = None
                    status_str = None

                    capf = os.path.join(ps_path, 'capacity')
                    if os.path.isfile(capf):
                        try:
                            with open(capf, 'r') as f:
                                percent = float(f.read().strip())
                        except Exception:
                            percent = None
                    else:
                        def _read_num(path):
                            try:
                                with open(path, 'r') as f:
                                    return float(f.read().strip())
                            except Exception:
                                return None
                        e_now = _read_num(os.path.join(ps_path, 'energy_now'))
                        e_full = _read_num(os.path.join(ps_path, 'energy_full'))
                        if e_now and e_full and e_full > 0:
                            percent = (e_now / e_full) * 100.0
                        else:
                            c_now = _read_num(os.path.join(ps_path, 'charge_now'))
                            c_full = _read_num(os.path.join(ps_path, 'charge_full'))
                            if c_now and c_full and c_full > 0:
                                percent = (c_now / c_full) * 100.0

                    for vfile in ['voltage_now', 'voltage_avg', 'voltage_min_design', 'voltage_max_design']:
                        vpath = os.path.join(ps_path, vfile)
                        if os.path.isfile(vpath):
                            try:
                                with open(vpath, 'r') as f:
                                    val = float(f.read().strip())
                                voltage_v = val / 1e6 if val > 1000 else val
                                break
                            except Exception:
                                continue

                    stpath = os.path.join(ps_path, 'status')
                    if os.path.isfile(stpath):
                        try:
                            with open(stpath, 'r') as f:
                                status_str = f.read().strip()
                        except Exception:
                            status_str = None

                    out = {
                        'percent': round(percent, 1) if isinstance(percent, float) else percent,
                        'voltage_v': round(voltage_v, 3) if isinstance(voltage_v, float) else voltage_v,
                        'status': status_str,
                        'power_supply': name
                    }
                    if any(v is not None for v in out.values()):
                        return out
        except Exception:
            pass
        return None

    def run_server(self):
        """Run the HTTP server"""
        server_address = ('', self.port)
        
        # Create custom request handler with access to the node
        server_node = self  # Reference to the ROS node
        
        class RequestHandler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            def end_headers(self):
                """Ensure connections are not kept alive to avoid HTTP/1.1 hangs when no Content-Length."""
                try:
                    self.send_header('Connection', 'close')
                except Exception:
                    pass
                super().end_headers()
            def address_string(self):  # Avoid reverse DNS blocking
                return self.client_address[0]
            def log_message(self, format, *args):  # Route logs to ROS logger without DNS lookup
                try:
                    server_node.get_logger().info("HTTP: " + (format % args))
                except Exception:
                    pass
            def _set_cors_headers(self):
                """Set CORS headers for cross-origin requests"""
                origin = self.headers.get('Origin')
                if origin:
                    # Echo specific origin when provided to support credentials
                    self.send_header('Access-Control-Allow-Origin', origin)
                    # Advise caches that these headers vary by request headers
                    self.send_header('Vary', 'Origin, Access-Control-Request-Method, Access-Control-Request-Headers')
                    self.send_header('Access-Control-Allow-Credentials', 'true')
                else:
                    # Fallback for non-CORS or tools like curl
                    self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                # Allow common headers and reflect requested headers if provided by the browser
                acrh = self.headers.get('Access-Control-Request-Headers')
                allow_headers = acrh if acrh else 'Content-Type, Authorization'
                self.send_header('Access-Control-Allow-Headers', allow_headers)
                self.send_header('Access-Control-Max-Age', '3600')
                # Allow private network access (Chrome PNA) when requested
                if (self.headers.get('Access-Control-Request-Private-Network', '').lower() == 'true'):
                    self.send_header('Access-Control-Allow-Private-Network', 'true')
            
            def do_OPTIONS(self):
                """Handle OPTIONS requests for CORS preflight"""
                self.close_connection = True
                # Log details to help debug any 501/blocked preflights
                try:
                    server_node.get_logger().info(
                        f"OPTIONS {self.path} | Origin={self.headers.get('Origin')} "
                        f"ACRM={self.headers.get('Access-Control-Request-Method')} "
                        f"ACRH={self.headers.get('Access-Control-Request-Headers')}"
                    )
                except Exception:
                    pass
                self.send_response(200)
                self._set_cors_headers()
                self.send_header('Allow', 'GET, POST, OPTIONS')
                self.send_header('Content-Length', '0')
                self.end_headers()
            
            def do_GET(self):
                """Handle GET requests"""
                self.close_connection = True
                parsed_path = urllib.parse.urlparse(self.path)
                
                if parsed_path.path == '/status':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    # Gather dynamic info
                    try:
                        wifi_info = server_node.get_wifi_info()
                    except Exception:
                        wifi_info = None
                    try:
                        battery_info = server_node.get_battery_info()
                    except Exception:
                        battery_info = None
                    
                    status = {
                        'status': 'running',
                        'hostname': socket.gethostname(),
                        'time': time.time(),
                        'streaming': server_node.streaming_active,
                        'mediamtx_url': server_node.mediamtx_url,
                        'mode': server_node.mode,
                        'notifications_enabled': server_node.notifications_enabled
                    }
                    status['signal'] = wifi_info.get('signal_dbm') if wifi_info else None
                    status['battery'] = battery_info.get('percent') if battery_info else None
                    
                    self.wfile.write(json.dumps(status).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    stream_status = server_node.camera_manager.get_stream_status()
                    stream_info = {
                        'url': server_node.mediamtx_url,
                        'active': stream_status['active'],
                        'status': stream_status['status']
                    }
                    self.wfile.write(json.dumps(stream_info).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path == '/stream/start':
                    # Start ffmpeg first (it will block on FIFO until writer starts), then signal camera_node to start FIFO writer
                    success = False
                    for _ in range(3):
                        success = server_node.camera_manager.start_camera_stream()
                        if success:
                            break
                        time.sleep(0.3)
                    if success:
                        # Now ask camera_node to start writing frames to FIFO
                        try:
                            msg = String(); msg.data = 'start_stream'
                            server_node.command_publisher.publish(msg)
                        except Exception:
                            pass
                    server_node.streaming_active = success
                    server_node.get_logger().info('Stream start requested via HTTP')
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    response = {
                        'status': 'success' if success else 'error',
                        'message': 'Streaming started' if success else 'Failed to start streaming',
                        'url': server_node.mediamtx_url
                    }
                    self.wfile.write(json.dumps(response).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path == '/stream/stop':
                    # Ask camera_node to stop FIFO writer first to avoid broken pipe spam
                    try:
                        msg = String(); msg.data = 'stop_stream'
                        server_node.command_publisher.publish(msg)
                    except Exception:
                        pass
                    time.sleep(0.2)
                    success = server_node.camera_manager.stop_camera_stream()
                    server_node.streaming_active = False
                    server_node.get_logger().info('Stream stop requested via HTTP')
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    response = {
                        'status': 'success' if success else 'error',
                        'message': 'Streaming stopped' if success else 'Failed to stop streaming'
                    }
                    self.wfile.write(json.dumps(response).encode())
                
                elif parsed_path.path == '/mode':
                    # Return current mode
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    response = {
                        'status': 'success',
                        'mode': server_node.mode,
                        'allowed_modes': ['armed', 'disarmed']
                    }
                    self.wfile.write(json.dumps(response).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path in ['/arm', '/mode/armed']:
                    # Arm detection and publish mode change
                    try:
                        server_node.mode = 'armed'
                        # Publish mode for other nodes
                        mode_msg = String(); mode_msg.data = server_node.mode
                        server_node.robot_mode_publisher.publish(mode_msg)
                        # Notify detection to arm
                        cmd_msg = String(); cmd_msg.data = 'arm'
                        server_node.command_publisher.publish(cmd_msg)
                        server_node.get_logger().info('System armed via HTTP')
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'success', 'mode': server_node.mode}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                    except Exception as e:
                        server_node.get_logger().error(f'Error arming system: {e}')
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                
                elif parsed_path.path in ['/disarm', '/mode/disarmed']:
                    # Disarm detection and publish mode change
                    try:
                        server_node.mode = 'disarmed'
                        mode_msg = String(); mode_msg.data = server_node.mode
                        server_node.robot_mode_publisher.publish(mode_msg)
                        cmd_msg = String(); cmd_msg.data = 'disarm'
                        server_node.command_publisher.publish(cmd_msg)
                        server_node.get_logger().info('System disarmed via HTTP')
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'success', 'mode': server_node.mode}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                    except Exception as e:
                        server_node.get_logger().error(f'Error disarming system: {e}')
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                
                elif parsed_path.path in ['/notifications', '/notifications/']:
                    # Return current notifications state
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    response = {
                        'status': 'success',
                        'enabled': server_node.notifications_enabled
                    }
                    self.wfile.write(json.dumps(response).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path in ['/notifications/on', '/notifications/off']:
                    # Toggle via GET convenience endpoints
                    new_state = parsed_path.path.endswith('/on')
                    server_node.notifications_enabled = new_state
                    # Publish Bool for other nodes
                    bool_msg = Bool()
                    bool_msg.data = new_state
                    server_node.notifications_publisher.publish(bool_msg)
                    server_node.get_logger().info(f'Notifications set to {new_state} via HTTP GET')
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    response = {'status': 'success', 'enabled': new_state}
                    self.wfile.write(json.dumps(response).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                elif parsed_path.path == '/recordings':
                    # List available recordings
                    try:
                        if not os.path.isdir(server_node.recordings_dir):
                            raise FileNotFoundError(f'Recordings dir not found: {server_node.recordings_dir}')
                        files = [f for f in os.listdir(server_node.recordings_dir) if f.endswith('.mp4')]
                        items = []
                        for f in files:
                            full_path = os.path.join(server_node.recordings_dir, f)
                            try:
                                stat = os.stat(full_path)
                                items.append({
                                    'name': f,
                                    'size_bytes': stat.st_size,
                                    'ctime': stat.st_ctime
                                })
                            except Exception:
                                # Skip files that cannot be stat'ed
                                continue
                        items.sort(key=lambda x: x['ctime'], reverse=True)
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'success', 'count': len(items), 'items': items}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                    except Exception as e:
                        server_node.get_logger().error(f'Error listing recordings: {e}')
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                
                elif parsed_path.path == '/recordings/latest':
                    # Return the latest recording metadata
                    try:
                        files = [f for f in os.listdir(server_node.recordings_dir) if f.endswith('.mp4')]
                        if not files:
                            raise FileNotFoundError('No recordings found')
                        latest = max(files, key=lambda f: os.path.getctime(os.path.join(server_node.recordings_dir, f)))
                        full_path = os.path.join(server_node.recordings_dir, latest)
                        stat = os.stat(full_path)
                        info = {
                            'name': latest,
                            'size_bytes': stat.st_size,
                            'ctime': stat.st_ctime,
                            'download_url': f'/recordings/{urllib.parse.quote(latest)}'
                        }
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'success', 'item': info}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                    except Exception as e:
                        server_node.get_logger().error(f'Error getting latest recording: {e}')
                        self.send_response(404)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                
                elif parsed_path.path.startswith('/recordings/'):
                    # Serve a specific recording file
                    try:
                        filename = parsed_path.path.split('/recordings/', 1)[1]
                        # Prevent path traversal
                        filename = os.path.basename(filename)
                        if not filename.endswith('.mp4'):
                            raise ValueError('Only .mp4 files are allowed')
                        full_path = os.path.join(server_node.recordings_dir, filename)
                        if not os.path.isfile(full_path):
                            raise FileNotFoundError('File not found')
                        file_size = os.path.getsize(full_path)
                        self.send_response(200)
                        self.send_header('Content-Type', 'video/mp4')
                        self.send_header('Content-Length', str(file_size))
                        self.send_header('Content-Disposition', f'inline; filename="{filename}"')
                        self._set_cors_headers()
                        self.end_headers()
                        with open(full_path, 'rb') as f:
                            while True:
                                chunk = f.read(64 * 1024)
                                if not chunk:
                                    break
                                self.wfile.write(chunk)
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                    except Exception as e:
                        server_node.get_logger().error(f'Error serving recording: {e}')
                        self.send_response(404)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                
                elif parsed_path.path == '/health':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    response = {'status': 'ok'}
                    self.wfile.write(json.dumps(response).encode())
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                
                else:
                    self.send_response(404)
                    self.send_header('Content-Type', 'text/plain')
                    self._set_cors_headers()
                    self.end_headers()
                    self.wfile.write(b'Not Found')
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
            
            def do_POST(self):
                """Handle POST requests"""
                self.close_connection = True
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                
                parsed_path = urllib.parse.urlparse(self.path)
                # Normalize path to avoid trailing slash mismatches
                norm_path = parsed_path.path.rstrip('/') if parsed_path.path != '/' else '/'
                
                try:
                    command_data = json.loads(post_data.decode('utf-8'))
                    
                    if norm_path == '/command':
                        command = command_data.get('command', '')
                        
                        # Handle streaming commands
                        if command == 'start_stream':
                            server_node.streaming_active = server_node.camera_manager.start_camera_stream()
                            server_node.get_logger().info('Stream start requested via command')
                        elif command == 'stop_stream':
                            server_node.camera_manager.stop_camera_stream()
                            server_node.streaming_active = False
                            server_node.get_logger().info('Stream stop requested via command')
                        
                        # Publish command to ROS topic
                        msg = String()
                        msg.data = command
                        server_node.command_publisher.publish(msg)
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        
                        response = {
                            'status': 'success',
                            'command': command
                        }
                        self.wfile.write(json.dumps(response).encode())
                    
                    elif norm_path == '/move':
                        linear_x = float(command_data.get('linear_x', 0.0))
                        angular_z = float(command_data.get('angular_z', 0.0))
                        
                        # Create and publish Twist message
                        twist = Twist()
                        twist.linear.x = linear_x
                        twist.angular.z = angular_z
                        server_node.cmd_vel_publisher.publish(twist)
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        
                        response = {
                            'status': 'success',
                            'linear_x': linear_x,
                            'angular_z': angular_z
                        }
                        self.wfile.write(json.dumps(response).encode())
                    
                    elif norm_path == '/mode':
                        mode = str(command_data.get('mode', '')).lower()
                        if mode not in ['armed', 'disarmed']:
                            raise ValueError("mode must be 'armed' or 'disarmed'")
                        server_node.mode = mode
                        # Publish mode to ROS topic
                        msg = String()
                        msg.data = mode
                        server_node.robot_mode_publisher.publish(msg)
                        # Also publish explicit arm/disarm command so detection_node toggles immediately
                        try:
                            cmd_msg = String()
                            cmd_msg.data = 'arm' if mode == 'armed' else 'disarm'
                            server_node.command_publisher.publish(cmd_msg)
                            server_node.get_logger().info(f"Mode set to {mode} via HTTP POST; sent command '{cmd_msg.data}'")
                        except Exception:
                            pass
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        
                        response = {
                            'status': 'success',
                            'mode': mode
                        }
                        self.wfile.write(json.dumps(response).encode())
                    
                    elif norm_path == '/notifications':
                        # Toggle notifications via POST { "enabled": true|false }
                        enabled = command_data.get('enabled', None)
                        if isinstance(enabled, str):
                            enabled = enabled.lower() in ['true', '1', 'yes', 'on']
                        if enabled is None:
                            raise ValueError("'enabled' must be provided and be a boolean")
                        server_node.notifications_enabled = bool(enabled)
                        bool_msg = Bool()
                        bool_msg.data = server_node.notifications_enabled
                        server_node.notifications_publisher.publish(bool_msg)
                        server_node.get_logger().info(f'Notifications set to {server_node.notifications_enabled} via HTTP POST')
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self._set_cors_headers()
                        self.end_headers()
                        response = {'status': 'success', 'enabled': server_node.notifications_enabled}
                        self.wfile.write(json.dumps(response).encode())
                    
                    else:
                        self.send_response(404)
                        self.send_header('Content-Type', 'text/plain')
                        self._set_cors_headers()
                        self.end_headers()
                        self.wfile.write(b'Not Found')
                
                except Exception as e:
                    server_node.get_logger().error(f'Error processing request: {str(e)}')
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json')
                    self._set_cors_headers()
                    self.end_headers()
                    
                    response = {
                        'status': 'error',
                        'message': str(e)
                    }
                    self.wfile.write(json.dumps(response).encode())
    
        # Start server
        try:
            httpd = ThreadingHTTPServer(server_address, RequestHandler)
            # Make handler threads daemon so they don't block shutdown
            try:
                httpd.daemon_threads = True
            except Exception:
                pass
            server_node.get_logger().info(f'HTTP server started on port {self.port}')
            httpd.serve_forever()
        except Exception as e:
            server_node.get_logger().error(f'Error starting HTTP server: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # Get MediaMTX server IP from environment or parameter
    import os
    mediamtx_server_ip = os.getenv('MEDIAMTX_SERVER_IP', '192.168.1.74')
    
    try:
        print(f"Starting robot server - MediaMTX server: {mediamtx_server_ip}")
        node = ServerNode(mediamtx_server_ip)
        
        # Start camera stream to MediaMTX server if requested
        if os.getenv('AUTO_START_STREAM', 'false').lower() == 'true':
            node.camera_manager.start_camera_stream()
            node.streaming_active = True
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down robot server...")
    except Exception as e:
        print(f"Error in server node: {str(e)}")
    finally:
        if 'node' in locals():
            if hasattr(node, 'camera_manager'):
                node.camera_manager.stop_camera_stream()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
