import time

previous = None
normal_speed = 255

def process_data(data, publish_fn=None):
    """
    Fungsi utama untuk memproses perubahan arah.
    """
    global previous

    if previous is None or data == previous:
        previous = data
        forward(publish_fn)
        return

    transition = (previous, data)

    transitions = {
        ('U', 'R'): [right, forward],
        ('U', 'D'): [turn_around, forward],
        ('U', 'L'): [left, forward],
        ('D', 'R'): [left, turn_around],
        ('D', 'L'): [right, turn_around],
        ('D', 'U'): [turn_around, forward],
        ('R', 'U'): [left, forward],
        ('R', 'L'): [turn_around, forward],
        ('R', 'D'): [right, forward],
        ('L', 'U'): [right, forward],
        ('L', 'D'): [left, forward],
        ('L', 'R'): [turn_around, forward],
    }

    if transition in transitions:
        for action in transitions[transition]:
            action(publish_fn)

    previous = data

def publish_speed(right_speed, left_speed, publish_fn):
    """
    Helper untuk publish data dan delay.
    """
    if publish_fn:
        publish_fn(f"{right_speed} {left_speed}")
    time.sleep(2)

def forward(publish_fn=None):
    publish_speed(normal_speed, normal_speed, publish_fn)

def right(publish_fn=None):
    publish_speed(-normal_speed // 2, normal_speed // 2, publish_fn)

def left(publish_fn=None):
    publish_speed(normal_speed // 2, -normal_speed // 2, publish_fn)

def turn_around(publish_fn=None):
    publish_speed(-normal_speed, normal_speed, publish_fn)

def idle(publish_fn=None):
    publish_speed(0, 0, publish_fn)
