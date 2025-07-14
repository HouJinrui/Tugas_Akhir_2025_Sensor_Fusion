import threading
import time

def twos_complement(val, bits=16):
    """Calculate two's complement for a signed value."""
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

class ModbusClient:
    def __init__(self, ip, client_id, data_queue):
        self.ip = ip
        self.client_id = client_id
        self.data_queue = data_queue
        self.client = None
        self.connected = False
        self.running = False
        self.thread = None
        self.last_successful_read = time.time()
        self.connection_timeout = 3.0
        self.consecutive_failures = 0
        self.max_failures = 3
        
        self.data = {
            'mpu_angle': 0.0,
            'shaft_angle': 0.0,
            'fused_angle': 0.0,
            'connected': False
        }
    
    def connect(self):
        try:
            if self.client:
                self.client.close()
            try:
                from pymodbus.client import ModbusTcpClient
            except ImportError:
                from pymodbus.client import ModbusTcpClient
            self.client = ModbusTcpClient(self.ip, port=502, timeout=2)
            self.connected = self.client.connect()
            if self.connected:
                self.last_successful_read = time.time()
                self.consecutive_failures = 0
                print(f"Connected to {self.ip}")
            else:
                print(f"Failed to connect to {self.ip}")
            return self.connected
        except Exception as e:
            print(f"Connection error for {self.ip}: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        self.connected = False
        self.consecutive_failures = 0
    
    def read_data(self):
        if not self.connected:
            return None
        from pymodbus.exceptions import ModbusException
        try:
            result = self.client.read_holding_registers(address=0, count=4)
            if result.isError():
                raise ModbusException("Modbus read error")
            elif not hasattr(result, 'registers') or len(result.registers) < 3:
                raise ModbusException("Invalid response or insufficient data")
            regs = result.registers
            self.data['mpu_angle'] = twos_complement(regs[0]) / 100.0
            self.data['shaft_angle'] = twos_complement(regs[1]) / 100.0
            self.data['fused_angle'] = twos_complement(regs[2]) / 100.0
            self.data['connected'] = True
            self.consecutive_failures = 0
            self.last_successful_read = time.time()
            return self.data.copy()
        except Exception as e:
            print(f"Read error for {self.ip}: {e}")
            self.consecutive_failures += 1
            if (self.consecutive_failures >= self.max_failures or 
                time.time() - self.last_successful_read > self.connection_timeout):
                self.connected = False
                print(f"Connection lost to {self.ip} (failures: {self.consecutive_failures})")
            self.data['connected'] = self.connected
            return None
    
    def start_reading(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def stop_reading(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def _read_loop(self):
        while self.running:
            if not self.connected:
                disconnected_data = self.data.copy()
                disconnected_data['connected'] = False
                self.data_queue.put((self.client_id, disconnected_data))
                if not self.connect():
                    time.sleep(2)
                    continue
            data = self.read_data()
            if data:
                self.data_queue.put((self.client_id, data))
            else:
                status_data = self.data.copy()
                status_data['connected'] = self.connected
                self.data_queue.put((self.client_id, status_data))
                if not self.connected:
                    self.disconnect()
            time.sleep(0.1)