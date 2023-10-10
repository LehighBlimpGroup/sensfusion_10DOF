import pygame
import time

class JoystickHandler:
    def __init__(self, blimp_type="bicopter", joy_index=0):
        self.blimp_type = blimp_type
        self.joy_index = joy_index
        
        self.joystick_init()
        
        # Internal joystick values
        self.right_vertical = 0
        self.right_horizontal = 0
        self.left_vertical = 0
        self.left_horizontal = 0

        # Internal button values initialized for (a, b, x, y, lb, rb)
        self.button_names = ['a', 'b', 'x', 'y', 'lb', 'rb']
        for btn in self.button_names:
            setattr(self, btn, 0)
            setattr(self, f"{btn}_old", 0)
            setattr(self, f"{btn}_state", 0)

        # Internal control values
        self.fx, self.fy, self.fz = 0, 0, 0
        self.tx, self.ty, self.tz = 0, 0, 0
        
        # Internal clock for dt
        self.time_start = time.time()

    def joystick_init(self):
        """Initialize the joystick."""
        pygame.joystick.init()
        while pygame.joystick.get_count() == 0:
            print("No controller Connected")
        pygame.display.init()
        self.joystick = pygame.joystick.Joystick(self.joy_index)
        self.joystick.init()

    def _update_button_states(self, btn_index):
        btn_name = self.button_names[btn_index]
        btn_val = getattr(self, btn_name)
        btn_old_val = getattr(self, f"{btn_name}_old")
        if btn_val == 1 and btn_old_val == 0:
            setattr(self, f"{btn_name}_state", not getattr(self, f"{btn_name}_state"))
        setattr(self, f"{btn_name}_old", btn_val)

    def update_joy_params(self):
        """Update joystick parameters."""
        pygame.event.pump()
        
        # Update button values and states
        for i, btn in enumerate(self.button_names):
            setattr(self, btn, self.joystick.get_button(i))
            self._update_button_states(i)
        
        # Update axis values with a dead-zone of 0.1
        axes = [('right_vertical', 3), ('right_horizontal', 2), ('left_horizontal', 0), ('left_vertical', 1)]
        for axis, idx in axes:
            val = self.joystick.get_axis(idx)
            setattr(self, axis, val if abs(val) > 0.1 else 0)

    def get_bicopter_controls(self):
        """Return controls for bicopter."""
        dt = time.time() - self.time_start
        self.time_start = time.time()

        self.fx = -1* self.right_vertical
        self.fz = self.fz + -1* self.left_vertical * dt if self.b_state else 0
        self.tz = -1* self.right_horizontal
        return [int(self.b_state), self.fx, self.fy, self.fz, self.tx, self.ty, self.tz, 0, 0, 0, 0, 0]

    def get_sblimp_controls(self):
        """Return controls for sblimp."""
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def get_outputs(self):
        """Get the output controls based on blimp type."""
        self.update_joy_params()
        if self.blimp_type == "bicopter":
            return self.get_bicopter_controls(), self.y_state

        

if __name__ == "__main__":
    # Create an instance of the JoystickHandler class
    handler = JoystickHandler()

    try:
        while True:
            outputs = handler.get_outputs()
            
            # Print the outputs
            if isinstance(outputs, tuple):
                rounded_values = [round(val, 2) for val in outputs[0]]  # rounding to 2 decimal places
                print((rounded_values, outputs[1]))
            else:
                rounded_values = [round(val, 2) for val in outputs]
                print(rounded_values)
            
            # Sleep for a short time to avoid flooding the console (optional, can be adjusted/removed)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("Loop terminated by user.")