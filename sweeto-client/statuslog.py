import time
import datetime

class StatusLog(object):
    def __init__(self):
        self.log = []
        self.current = self.create_entry("Initial")

    def create_entry(self, state, end=None):
        return dict(state = state,
                    start = time.time(),
                    end = end,
                    updated = 0,
                    lwheeldist=0,
                    rwheeldist=0)
        
    def update(self, status):
        state = status.get("state")
        duration = time.time() - self.current["updated"]
        ldist = status["motors"].get("LeftWheel_Speed") * duration
        rdist = status["motors"].get("RightWheel_Speed") * duration
        self.current["lwheeldist"] += ldist
        self.current["rwheeldist"] += rdist
        self.current["updated"] = time.time()

        if state != self.current.get("state"):
            new = self.create_entry(state)
            self.current["end"] = time.time()
            self.current = new
            self.log.append(new)
            print "Updated status log: [%s]" % self.print_entry(self.current)
            if len(self.log) > 1:
                print "Prevous status log: [%s]" % self.print_entry(self.log[-2])


    def print_entry(self, entry):
        start_time = time.strftime("%H:%M:%S", time.localtime(entry["start"]))
        duration = datetime.timedelta(seconds=((entry["end"] or time.time()) - entry["start"]))
        
        txt = "State: %s Started: %s Duration: %s Drove: %smm / %smm" % (entry["state"],
                                                                         start_time,
                                                                         duration,
                                                                         entry["lwheeldist"],
                                                                         entry["rwheeldist"])
        return txt
