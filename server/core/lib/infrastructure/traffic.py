from core.lib.infrastructure.utils import constants as tfc


class TrafficController:

    def __init__(self):
        # self.signals = signals  # signals is a dict with keys in {0,1,..,7}
        self.__no_phases = tfc.NO_OF_PHASES
        self.__cycle_length = tfc.TRAFFIC_SIGNAL_CYCLE
        self.__table = self.generate_random_table()
        self.__dt = tfc.TRAFFIC_CONTROLLER_DT
        self.signals = dict.fromkeys(list(range(8)))
        self.__set_signals()

    def __set_signals(self):
        self.signals = dict.fromkeys(range(12))
        for i in range(12):
            self.signals[i] = TrafficSignal(tcid=i)

    def set_table(self, table):
        self.__table = table

    def generate_random_table(self):
        table = {}
        gt = 4   #green time
        at = 1   #amber time
        rt = self.__cycle_length - gt - at

        table[0] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        table[1] = ['r', gt+at, 'g', 2*gt + at, 'a', 2*(gt+at), 'r', self.__cycle_length]
        table[6] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        table[7] = ['r', gt+at, 'g', 2*gt + at, 'a', 2*(gt+at), 'r', self.__cycle_length]
        table[3] = ['r', 2*(gt+at), 'g', 2*(gt+at)+gt, 'a', at, 'r', self.__cycle_length]
        table[4] = ['r', 3*(gt+at), 'g', 3*(gt+at)+gt, 'a', self.__cycle_length]
        table[9] = ['r', 2*(gt+at), 'g', 2*(gt+at)+gt, 'a', at, 'r', self.__cycle_length]
        table[10] = ['r', 3*(gt+at), 'g', 3*(gt+at)+gt, 'a', self.__cycle_length]


        # ToDo: add constants for 'g','r','y'
        # table[0] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[1] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[6] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[7] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[3] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[4] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[9] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[10] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]

        table[2] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[5] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[8] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[11] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]

        return table

    def print_table(self):
        for i in self.__table.keys():
            print(i, self.__table[i])

    def update_signals(self):
        for signal in self.signals.values():
            signal.tick += 1
            # reset cycle
            # print(self.__dt, self.__cycle_length)
            if signal.tick * self.__dt > self.__cycle_length:
                signal.tick = 0

            for c_ind in range(0, len(self.__table[signal.tcid]), 2):
                # print(self.__table[signal.tcid][c_ind+1])
                if signal.tick * self.__dt < self.__table[signal.tcid][c_ind+1]:
                    color = self.__table[signal.tcid][c_ind]
                    time_remaining = self.__table[signal.tcid][c_ind+1] - signal.tick * self.__dt
                    signal.set_state(color, time_remaining)
                    break

    def reset_signals(self):
        for signal in self.signals.values():
            signal.tick = 0
            signal.color = None
            signal.time_remaining = None


class TrafficSignal:

    def __init__(self, tcid):
        self.color = None
        self.time_remaining = None
        self.tcid = tcid
        self.node = None
        self.tick = 0

    def set_node(self, node):
        self.node = node

    def set_state(self, color, time_remaining):
        self.color = color
        self.time_remaining = time_remaining

    def get_state(self):
        return [self.color, self.time_remaining]
