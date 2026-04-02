def __init__(self):
    super().__init__('tilt_control_node')
    
    if not _OK:
        self.get_logger().fatal('freenect not available.')
        raise RuntimeError('freenect unavailable')
    
    self.declare_parameter('device_index', 0)  # Add this
    self.declare_parameter('publish_rate_hz', 5.0)
    
    self._dev = self.get_parameter('device_index').value  # Add this
    rate = self.get_parameter('publish_rate_hz').value
