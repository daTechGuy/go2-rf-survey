class Trigger:
    class Request:
        pass

    class Response:
        def __init__(self):
            self.success, self.message = False, ""
