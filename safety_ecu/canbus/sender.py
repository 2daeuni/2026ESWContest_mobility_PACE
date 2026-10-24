import can


class CANSender:
    def __init__(self, channel="can0"):
        self.channel = channel
        self.bus = None
        self.tx_error_reported = False

    def open(self):
        self.bus = can.Bus(
            interface="socketcan",
            channel=self.channel,
        )
        print(f"[CAN] {self.channel} opened for TX")
        
    def send(self, arbitration_id, data):
        if self.bus is None:
            raise RuntimeError("CAN bus is not opened")

        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

        try:
            self.bus.send(message)

        except can.CanError as error:
            if not self.tx_error_reported:
                print(
                    "[CAN TX] "
                    f"TRANSMISSION ERROR: {error}"
                )

                self.tx_error_reported = True

            return False

        if self.tx_error_reported:
            print(
                "[CAN TX] "
                "TRANSMISSION RECOVERED"
            )

            self.tx_error_reported = False

        return True

    def send_haptic_command(
        self,
        risk_level,
        vibration_command,
        intensity,
        frequency,
        alive_counter,
    ):
        data = bytes([
            risk_level & 0xFF,
            vibration_command & 0xFF,
            intensity & 0xFF,
            frequency & 0xFF,
            0x00,
            0x00,
            0x00,
            alive_counter & 0xFF,
        ])

        return self.send(
            arbitration_id=0x110,
            data=data,
        )

    def send_motor_command(
        self,
        control_mode,
        output_limit,
        risk_level,
        command_flags,
        alive_counter,
    ):
        data = bytes([
            control_mode & 0xFF,
            output_limit & 0xFF,
            risk_level & 0xFF,
            command_flags & 0xFF,
            0x00,
            0x00,
            0x00,
            alive_counter & 0xFF,
        ])

        self.send(
            arbitration_id=0x120,
            data=data,
        )

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            print(f"[CAN] {self.channel} TX closed")


if __name__ == "__main__":
    sender = CANSender("can0")

    try:
        sender.open()

        sender.send_motor_command(
            control_mode=1,
            output_limit=30,
            risk_level=3,
            command_flags=1,
            alive_counter=57,
        )

        print(
            "[CAN TX] "
            "MOTOR_COMMAND "
            "ID=0x120"
        )

    except KeyboardInterrupt:
        print("\n[CAN TX] Stopped by user")

    finally:
        sender.close()
