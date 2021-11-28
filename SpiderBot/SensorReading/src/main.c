#include <stdio.h>
#include <stdint.h>
#include "helper.h"
#include "metal/i2c.h"
#include "metal/gpio.h"


struct metal_gpio *ledHandler;
int led[3];


int main(){
    //set_up_I2C()
    
    
    ledHandler = metal_gpio_get_device(0);

    led[0] = 19;
    led[1] = 21;
    led[2] = 22;
    for(int i = 0; i < 3; i++){
        metal_gpio_set_pin(ledHandler,led[i],1);
        metal_gpio_enable_output(ledHandler,led[i]);
        delay(1000);
    }
    
    for(int i = 0; i < 3; i++){
        metal_gpio_set_pin(ledHandler,led[i],0);
        delay(500);
        metal_gpio_set_pin(ledHandler,led[i],1);
        delay(500);
    }
    for(int i = 0; i < 3; i++){
        metal_gpio_disable_output(ledHandler,led[i]);
        delay(1000);
    }

    
    

    



    return(0);
}