import sys
import os
import io

# Add the c_library_v2 path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'external', 'c_library_v2', 'python'))

from pymavlink import mavutil

class MAVLinkDecoder:
    def __init__(self):
        # Create a dummy file object for MAVLink
        self.mav = mavutil.mavlink.MAVLink(io.BytesIO(), srcSystem=1, srcComponent=1)
        
    def decode_hex_message(self, hex_string):
        """
        Decode an ASCII hex string into a MAVLink message.
        hex_string: space-separated ASCII hex bytes (e.g., "FE 09 00 01 00 00 00 00 00 00 00 00 00 00 00")
        Returns: (message_name, decoded_fields) or None if invalid
        """
        try:
            # Convert hex string to bytes
            hex_string = hex_string.strip()
            if not hex_string:
                return None
                
            # Split by spaces and convert each pair of hex characters to a byte
            hex_parts = hex_string.split()
            if len(hex_parts) == 0:
                return None
            
            # Convert hex strings to actual bytes
            data = bytearray()
            for hex_byte in hex_parts:
                data.append(int(hex_byte, 16))
            
            data = bytes(data)
            
            # MAVLink messages start with 0xFE or 0xFD
            if len(data) < 3 or (data[0] != 0xFE and data[0] != 0xFD):
                return None
            
            # Parse the message by feeding bytes one at a time
            msg = None
            for byte in data:
                msg = self.mav.parse_char(bytes([byte]))
                if msg:
                    break
            
            if msg is None:
                return None
            
            # Extract message info
            msg_name = msg.get_type()
            msg_dict = msg.to_dict()
            
            # Format nicely
            formatted = f"{msg_name}"
            for key, value in msg_dict.items():
                if key != 'mavpackettype':
                    formatted += f"\n  {key}: {value}"
            
            return (msg_name, formatted)
        except Exception as e:
            return None
    
    def decode_message_batch(self, hex_messages):
        """
        Decode multiple hex messages.
        hex_messages: list of hex strings
        Returns: list of (message_name, formatted_string) tuples
        """
        results = []
        for hex_msg in hex_messages:
            decoded = self.decode_hex_message(hex_msg)
            if decoded:
                results.append(decoded)
        return results
