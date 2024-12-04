
def start_flask_server(self):

    def run(self):
        """Main loop."""
        while self.step(32) != -1:  # Run Webots simulation at 32 ms steps
            self.apply_hand_angles()
