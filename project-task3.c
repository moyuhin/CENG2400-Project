 for(i=0;i<21;i++){
        v=GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6);
        if(v==0) {
        // You need to change here.
            print7("l90");        //rotate left around 90 degree
            wait_for_base_ready();

            print7("f050");       //move straight line leftward around 50cm
            wait_for_base_ready();

            print7("r90");         //rotate right around 90 degree
            wait_for_base_ready();
          
            print7("f050");       //move straight line forward around 50cm
            wait_for_base_ready();
            i = i + 5;
            
            print7("r90")          //rotate right around 90 degree
            wait_for_base_ready();
          
            print7("f050");       //move straight line rightward around 50cm
            wait_for_base_ready();
            
            print7("l90");        //rotate left around 90 degree
            wait_for_base_ready();        

        }
        print7("f010");
        wait_for_base_ready();
        s++;
    }
