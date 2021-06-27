#include "sensor_head.h"

int main()
{
    //Initialization
    InitConsole();
    InitCanbus();
    InitI2C();
     i=0;
    // menu();
    UARTprintf("Events - \n");
    while(1)
    {

        //Branching was done here
        if(string_ready==1)
        {
            string_ready=0;
            UARTprintf("\n");
            do_branch(str);
            for(i=0;i<10;i++)           //To clear the buffer of last iteration
              {str[i]='\0';}
              i=0;
        }

        //Receiving Can data if available
        receive_can();
        if(new_can_recv)
            send_reading();
    }


    return 0;
}
