"""

State machine to handle requests processing.

"""

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.currentState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_state(self, name):
        self.currentState = name.upper()

    def run(self, tokens):
        handler = self.handlers[self.currentState]
        self.currentState = handler(tokens).upper()
