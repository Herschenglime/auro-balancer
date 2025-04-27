# quick chatgpt generated helper to wait for i2c to be ready to prevent bus contention on
# startup
import time
import board
import busio


def wait_for_i2c_ready(timeout=5.0, retry_delay=0.1):
    start_time = time.time()
    while True:
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            while not i2c.try_lock():
                time.sleep(retry_delay)
            i2c.unlock()
            return
        except Exception as e:
            if time.time() - start_time > timeout:
                raise RuntimeError("I2C bus did not become ready in time.") from e
            time.sleep(retry_delay)
