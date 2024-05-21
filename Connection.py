class Connection:
    def __init__(self, name: str) -> None:
        self.name = name
        self.address = ""


def send_message(receiver: str, message: str):
    print(f"To {receiver}: {message}")
