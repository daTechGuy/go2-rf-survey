import time

import fakeloop


class _Now:
    def __init__(self):
        self.nanoseconds = time.time_ns()


class _Clock:
    def now(self):
        return _Now()


class _Future:
    def __init__(self, result):
        self._r = result

    def result(self):
        return self._r

    def add_done_callback(self, cb):
        cb(self)


class _Client:
    def __init__(self, srv_type, name):
        self.srv_type, self.service = srv_type, name.rsplit("/", 1)[-1]

    def call_async(self, _req):
        ok, msg = fakeloop.service(self.service)
        r = self.srv_type.Response()
        r.success, r.message = ok, msg
        return _Future(r)


class Node:
    def __init__(self, name):
        self._name = name
        self._subs = {}
        self._timers = []

    def create_subscription(self, _type, topic, cb, _qos):
        self._subs.setdefault(topic, []).append(cb)

    def create_client(self, srv_type, name):
        return _Client(srv_type, name)

    def create_timer(self, period, cb):
        self._timers.append([time.time() + period, period, cb])

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass
