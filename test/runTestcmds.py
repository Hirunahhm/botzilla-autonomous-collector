import KobukiDriver as kobuki
import time

test_cmds = ["w", "a", "d", "x", "q"]

def main():
    time.sleep(2)


    my_kobuki = kobuki.Kobuki()

    # Play start up sound
    #   my_kobuki.play_on_sound()

    for cmd in test_cmds:
        key = cmd
        if key == "w":
            # Move forward
            my_kobuki.move(200, 200, 0)
        elif key == "s":
            # Move backward
            my_kobuki.move(-200, -200, 0)
        elif key == "a":
            # Turn left
            my_kobuki.move(100, -100, 0)
        elif key == "d":
            # Turn right
            my_kobuki.move(-100, 100, 0)
        elif key == "x":
            # Stop
            my_kobuki.move(0, 0, 0)
        elif key == "1":
            # Play sound
            my_kobuki.play_button_sound()
        elif key == "2":
            # LED Control
            my_kobuki.set_led1_green_colour()
            time.sleep(1)
            my_kobuki.set_led2_red_colour()
            time.sleep(1)
            my_kobuki.clr_led1()
            my_kobuki.clr_led2()
        elif key == "q":
            # Quit
            break

        time.sleep(1)
        # Print sensor data
        print(my_kobuki.encoder_data())


if __name__ == "__main__":
    main()