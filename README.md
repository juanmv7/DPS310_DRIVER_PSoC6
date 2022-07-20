# DPS310_DRIVER_PSoC6

Driver for DPS310 using a PSoC6-BLE-CY8CKIT-062-BLE. I include here in this repository an example to read the temperature and the pressure measured by the sensor. You can use the library to create another _main.c_ with another configuration. 

## Differents configurations

In the library you will see I used an standard configuration. However, you can change the bytes you send to the DPS310 to use another configuration. To learn more about these differents options, may check the [DPS310 Datasheet](https://www.infineon.com/dgdl/Infineon-DPS310-DS-v01_00-EN.pdf?fileId=5546d462576f34750157750826c42242).

## Using ModusToolBox

You may use the infineon software [Modustoolbox](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) to run this project. If you are going to to do this, you will not need to create a new ModusToolbox apllication and directly import the project.

**Note: You will need to set a environment variable in your computer called CY_TOOLS_PATHS to the path where modustoolbox is installed and to the folder tools_2.4 like this:**

C:/Users/<Name>/ModusToolBox/tools_2.4/


### Debug UART Port

You can use PuTTY to run a window in the COMX (_Check the device manager_) and with a baudrate of 115200.


### PSoC64_commands_for_keys: **CY8CKIT-064B0S2-4343W**

- cysecuretools -t cy8ckit-064b0s2-4343w init

- cysecuretools -t cy8ckit-064b0s2-4343w -p policy/policy_single_CM0_CM4_swap.json create-keys

- cysecuretools -t cy8ckit-064b0s2-4343w -p policy/policy_single_CM0_CM4_swap.json provision-device

- cysecuretools -t cy8ckit-064b0s2-4343w -p policy/policy_single_CM0_CM4_swap.json re-provision-device 

_**Dont forget to turn on the LED in parpadeo and to remove the jumper**_
