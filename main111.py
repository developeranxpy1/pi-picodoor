"""
===============================================================================
PICO DOOR LOCK SYSTEM - v2.0 REFACTORED
===============================================================================

NOMINAL STATE: Door Closed (Pin 19 Low) is the default/safe state.

FEATURES:
1. Keypad Entry: 4-digit code (default: 3301) for secure access.
2. Proximity Unlock: Ultrasonic sensor for hands-free entry.
3. Instant Auto-Lock: Locks when door closes.
4. Jam Detection: Feedback loop with 3x sweep clearing.
5. Edit Mode: Change code via keypad (code: 911 + ENTER).

PIN FUNCTIONS:
- GP0-GP8  : Keypad 1-9
- GP9      : Reset Lock
- GP10     : Zero (0)
- GP11     : Enter
- GP12     : Star (*) Force Lock
- GP13     : Buzzer
- GP14     : Green LED
- GP15     : Red LED
- GP16     : Ultrasonic ECHO
- GP17     : Ultrasonic TRIG
- GP18     : Servo
- GP19     : Door Sensor (High=OPEN, Low=CLOSED)
- GP21     : Lock Sensor (High=LOCKED)
===============================================================================
"""

from machine import Pin, PWM
from time import sleep, ticks_ms, ticks_us

# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

# Timing (milliseconds)
DEBOUNCE_MS = 150
GRACE_PERIOD_MS = 1500
RESYNC_COOLDOWN_MS = 2000
AUTO_LOCK_DELAY_MS = 3000
FAILSAFE_LOCK_MS = 5000
SERVO_IDLE_TIMEOUT_MS = 10000
SENSOR_HOLD_MS = 300  # 0.3 seconds minimum presence to avoid auto-trigger
SENSOR_UNLOCK_MIN_MS = 1500  # 1.5 seconds minimum unlock duration for proximity
ULTRASONIC_TIMEOUT_US = 30000
KEYPAD_COOLDOWN_MS = 5000  # Lockout after max attempts
WATCHDOG_TIMEOUT_MS = 5000

# Servo Angles
LOCK_ANGLE = 180
UNLOCK_ANGLE = 0

# Proximity (centimeters)
PROXIMITY_IN_CM = 25  # +5cm detection range
PROXIMITY_OUT_CM = 30
EMA_ALPHA = 0.5

# Limits
MAX_ATTEMPTS = 3
MAX_RESYNC_RETRIES = 5
ALARMED_MARKER = 6  # Used to mark alarm triggered

# Codes
DEFAULT_CODE = [3, 3, 0, 1]
EDIT_CODE = [9, 1, 1]

# Debug
DEBUG_ENABLED = True
DEBUG_VERBOSITY = 2

# =============================================================================
# STATE ENUMS
# =============================================================================

class LockTarget:
    LOCKED = True
    UNLOCKED = False

class UnlockSource:
    NONE = None
    KEYPAD = "keypad"
    SENSOR = "sensor"

# =============================================================================
# SYSTEM STATE (replaces globals)
# =============================================================================

class State:
    def __init__(self):
        self.main_code = DEFAULT_CODE.copy()
        self.user_input = []
        self.attempts = 0
        self.in_edit_mode = False
        self.keypad_locked_until = 0
        
        self.unlock_time = None
        self.unlock_source = UnlockSource.NONE
        
        self.target_locked = LockTarget.UNLOCKED
        self.last_door_state = 0
        
        self.sensor_enter_time = None
        self.filtered_dist = 100.0
        
        self.last_servo_move = ticks_ms()
        self.stuck_timer_start = None
        self.last_resync_time = 0
        self.resync_retry_count = 0
        
        self.last_loop_tick = ticks_ms()

state = State()

# =============================================================================
# HARDWARE SETUP
# =============================================================================

buzzer = PWM(Pin(13))
buzzer.freq(2000)

led_green = Pin(14, Pin.OUT)
led_red = Pin(15, Pin.OUT)

servo = PWM(Pin(18))
servo.freq(50)

TRIG = Pin(17, Pin.OUT)
ECHO = Pin(16, Pin.IN)

buttons = [Pin(i, Pin.IN, Pin.PULL_UP) for i in range(9)]
reset_lock_button = Pin(9, Pin.IN, Pin.PULL_UP)
zero_button = Pin(10, Pin.IN, Pin.PULL_UP)
enter_button = Pin(11, Pin.IN, Pin.PULL_UP)
clear_button = Pin(12, Pin.IN, Pin.PULL_UP)

DOOR_SENSOR = Pin(19, Pin.IN, Pin.PULL_DOWN)
LOCK_SENSOR = Pin(21, Pin.IN, Pin.PULL_DOWN)

# =============================================================================
# DEBUG LOGGING
# =============================================================================

def log(msg, level=1):
    if DEBUG_ENABLED and level <= DEBUG_VERBOSITY:
        print(f"[{ticks_ms()}] {msg}")

# =============================================================================
# ULTRASONIC SENSOR
# =============================================================================

def measure_distance():
    """Measure distance in cm. Returns 999.0 on timeout."""
    TRIG.low()
    sleep(0.000002)
    TRIG.high()
    sleep(0.00001)
    TRIG.low()
    
    timeout_start = ticks_us()
    while ECHO.value() == 0:
        if ticks_us() - timeout_start > ULTRASONIC_TIMEOUT_US:
            return 999.0
    pulse_start = ticks_us()
    
    while ECHO.value() == 1:
        if ticks_us() - pulse_start > ULTRASONIC_TIMEOUT_US:
            return 999.0
    pulse_end = ticks_us()
    
    duration = pulse_end - pulse_start
    return (duration * 0.0343) / 2

# =============================================================================
# SERVO CONTROL
# =============================================================================

def angle_to_us(angle, min_us=500, max_us=2500):
    angle = max(0, min(180, angle))
    return int(min_us + (max_us - min_us) * angle / 180)

def set_servo_angle(angle):
    try:
        ns = angle_to_us(angle) * 1000
        servo.duty_ns(ns)
        state.last_servo_move = ticks_ms()
        log(f"Servo -> {angle}°", 2)
        return True
    except Exception as e:
        log(f"Servo error: {e}", 1)
        return False

def servo_lock():
    log("Commanding Lock")
    state.target_locked = LockTarget.LOCKED
    return set_servo_angle(LOCK_ANGLE)

def servo_unlock():
    log("Commanding Unlock")
    state.target_locked = LockTarget.UNLOCKED
    return set_servo_angle(UNLOCK_ANGLE)

# =============================================================================
# AUDIO & LED FEEDBACK
# =============================================================================

def beep(duration=0.05, freq=2000):
    buzzer.freq(freq)
    buzzer.duty_u16(60000)
    sleep(duration)
    buzzer.duty_u16(0)

def success_beep():
    beep(0.1, 1500)
    sleep(0.05)
    beep(0.1, 2000)
    sleep(0.05)
    beep(0.15, 2500)

def error_beep():
    beep(0.2, 1000)

def lock_sound():
    beep(0.15, 800)
    sleep(0.05)
    beep(0.15, 700)
    sleep(0.05)
    beep(0.15, 600)

def malfunction_beep(duration=10.0):
    start = ticks_ms()
    while (ticks_ms() - start) < (duration * 1000):
        for freq in range(500, 2500, 200):
            buzzer.freq(freq)
            buzzer.duty_u16(60000)
            sleep(0.02)
        for freq in range(2500, 500, -200):
            buzzer.freq(freq)
            buzzer.duty_u16(60000)
            sleep(0.02)
    buzzer.duty_u16(0)

def blink_red(times=1):
    for _ in range(times):
        led_red.on()
        sleep(0.1)
        led_red.off()
        sleep(0.1)

# =============================================================================
# LOCK/UNLOCK SYSTEM
# =============================================================================

def reset_input():
    state.user_input = []
    state.attempts = 0
    state.in_edit_mode = False

def unlock_system(source, reason=""):
    print(f">>> UNLOCK [{source}] {reason}")
    
    if source == UnlockSource.SENSOR:
        servo_unlock()
    else:
        led_green.on()
        led_red.off()
        servo_unlock()
        success_beep()
    
    state.unlock_time = ticks_ms()
    state.unlock_source = source

def lock_system(from_keypad=False, reason=""):
    if DOOR_SENSOR.value():
        print(f">>> LOCK BLOCKED: Door is open")
        state.target_locked = LockTarget.UNLOCKED
        return False
    
    source = "keypad" if from_keypad else "auto"
    print(f">>> LOCK [{source}] {reason}")
    
    servo_lock()
    led_green.off()
    
    if from_keypad:
        blink_red(1)
        lock_sound()
    
    reset_input()
    state.unlock_time = None
    state.unlock_source = UnlockSource.NONE
    state.sensor_enter_time = None
    return True

# =============================================================================
# KEYPAD HANDLING
# =============================================================================

def is_keypad_locked():
    # Locked if: cooldown active, door open, or sensor unlock active
    if ticks_ms() < state.keypad_locked_until:
        return True
    if DOOR_SENSOR.value():
        return True
    if state.unlock_source == UnlockSource.SENSOR:
        return True
    return False

def check_code():
    if state.user_input == state.main_code:
        unlock_system(UnlockSource.KEYPAD, "Correct code entered")
    else:
        state.attempts += 1
        log(f"Wrong code! Attempt {state.attempts}/{MAX_ATTEMPTS}")
        error_beep()
        if state.attempts >= MAX_ATTEMPTS:
            state.keypad_locked_until = ticks_ms() + KEYPAD_COOLDOWN_MS
            lock_system(reason="Max attempts exceeded")
        else:
            state.user_input.clear()

def read_buttons():
    if is_keypad_locked():
        return
    for i, button in enumerate(buttons):
        if not button.value():
            state.user_input.append(i + 1)
            led_green.on()
            beep(0.03, 1800)
            led_green.off()
            sleep(DEBOUNCE_MS / 1000)
            break

def handle_special_buttons():
    if is_keypad_locked():
        return
    
    # Star (*) = Force lock
    if not clear_button.value():
        state.user_input.clear()
        lock_system(from_keypad=True, reason="Star button pressed")
        sleep(DEBOUNCE_MS / 1000)
        return
    
    if not reset_lock_button.value():
        lock_system(from_keypad=True, reason="Reset button pressed")
        sleep(DEBOUNCE_MS / 1000)
        return
    
    if not zero_button.value():
        state.user_input.append(0)
        led_green.on()
        beep(0.03, 1800)
        led_green.off()
        sleep(DEBOUNCE_MS / 1000)
    
    if not enter_button.value():
        # Gate EDIT mode behind ENTER
        if state.user_input == EDIT_CODE and not state.in_edit_mode:
            state.in_edit_mode = True
            state.user_input.clear()
            log("Edit mode activated")
            beep(0.2, 2500)
        elif state.in_edit_mode:
            if len(state.user_input) == 4:
                state.main_code = state.user_input.copy()
                state.user_input.clear()
                state.in_edit_mode = False
                success_beep()
                log(f"New code set: {state.main_code}")
            else:
                error_beep()
        else:
            check_code()
        sleep(DEBOUNCE_MS / 1000)

# =============================================================================
# BACKGROUND MONITORS
# =============================================================================

def handle_servo_timeout():
    if ticks_ms() - state.last_servo_move >= SERVO_IDLE_TIMEOUT_MS:
        servo.duty_ns(0)

def handle_lock_feedback():
    if DOOR_SENSOR.value():
        state.stuck_timer_start = None
        state.resync_retry_count = 0
        return
    
    actual_locked = LOCK_SENSOR.value() == 1
    
    if actual_locked != state.target_locked:
        if state.stuck_timer_start is None:
            state.stuck_timer_start = ticks_ms()
            state.resync_retry_count = 0
        
        elapsed = ticks_ms() - state.stuck_timer_start
        
        if elapsed > GRACE_PERIOD_MS and (ticks_ms() - state.last_resync_time) > RESYNC_COOLDOWN_MS:
            if state.resync_retry_count < MAX_RESYNC_RETRIES:
                log(f"Jam detected! Attempt {state.resync_retry_count + 1}/{MAX_RESYNC_RETRIES}")
                state.last_resync_time = ticks_ms()
                state.resync_retry_count += 1
                
                for _ in range(3):
                    set_servo_angle(UNLOCK_ANGLE)
                    sleep(0.4)
                    set_servo_angle(LOCK_ANGLE)
                    sleep(0.4)
                
                target_angle = LOCK_ANGLE if state.target_locked else UNLOCK_ANGLE
                set_servo_angle(target_angle)
            elif state.resync_retry_count == MAX_RESYNC_RETRIES:
                log("MALFUNCTION: Max retries exceeded!")
                malfunction_beep(10.0)
                state.resync_retry_count = ALARMED_MARKER
    else:
        state.stuck_timer_start = None
        state.resync_retry_count = 0

def handle_autolock():
    if state.unlock_time is None:
        return
    
    elapsed = ticks_ms() - state.unlock_time
    
    if state.unlock_source == UnlockSource.KEYPAD and elapsed >= AUTO_LOCK_DELAY_MS:
        lock_system(reason="Auto-lock timer expired")
    elif elapsed >= FAILSAFE_LOCK_MS:
        log("Failsafe lock triggered")
        lock_system(reason="Failsafe timeout")

def handle_sensor_logic():
    raw_dist = measure_distance()
    
    if raw_dist < 400:
        if state.filtered_dist > 99:
            state.filtered_dist = raw_dist
        state.filtered_dist = (EMA_ALPHA * raw_dist) + (1 - EMA_ALPHA) * state.filtered_dist
    
    if state.filtered_dist < PROXIMITY_IN_CM:
        if state.sensor_enter_time is None:
            log(f"Proximity entry: {state.filtered_dist:.1f}cm")
            state.sensor_enter_time = ticks_ms()
        elif (ticks_ms() - state.sensor_enter_time) >= SENSOR_HOLD_MS:
            if state.unlock_time is None and state.unlock_source != UnlockSource.KEYPAD:
                unlock_system(UnlockSource.SENSOR, "Proximity hold detected")
    elif state.filtered_dist > PROXIMITY_OUT_CM:
        if state.sensor_enter_time is not None:
            log(f"Proximity exit: {state.filtered_dist:.1f}cm")
        state.sensor_enter_time = None
        if state.unlock_source == UnlockSource.SENSOR:
            # Only lock if minimum unlock duration has passed
            if state.unlock_time and (ticks_ms() - state.unlock_time) >= SENSOR_UNLOCK_MIN_MS:
                lock_system(reason="Person left proximity")

def handle_door_sensor():
    current = DOOR_SENSOR.value()
    
    if not current and state.last_door_state:
        lock_system(reason="Door closed")
    
    if current:
        if state.target_locked == LockTarget.LOCKED:
            servo_unlock()
        if state.unlock_time is not None:
            state.unlock_time = ticks_ms()
    
    state.last_door_state = current

def check_watchdog():
    if ticks_ms() - state.last_loop_tick > WATCHDOG_TIMEOUT_MS:
        log("WATCHDOG: Loop stalled!")
        # Could add machine.reset() here
    state.last_loop_tick = ticks_ms()

# =============================================================================
# MAIN
# =============================================================================

def setup():
    reset_input()
    state.last_door_state = DOOR_SENSOR.value()
    
    # Boot sound - cheerful ascending tones
    beep(0.08, 1000)
    beep(0.08, 1500)
    beep(0.08, 2000)
    beep(0.15, 2500)
    
    if state.last_door_state:
        log("Boot: Door open, staying unlocked")
        state.target_locked = LockTarget.UNLOCKED
        set_servo_angle(UNLOCK_ANGLE)
    else:
        lock_system(reason="Boot with door closed")

def main():
    setup()
    
    while True:
        check_watchdog()
        read_buttons()
        handle_special_buttons()
        handle_autolock()
        handle_sensor_logic()
        handle_servo_timeout()
        handle_lock_feedback()
        handle_door_sensor()
        sleep(0.02)

if __name__ == "__main__":
    main()
