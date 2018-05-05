
#include "cli.h"


void updateCLI( cliHdlr_t *cli, refValue_t *ref){

	    Serial.println(cli->BufferSerialInput);
	    //Process the received string
	    if(cli->BufferSerialInput[0]=='#'){
	    	if(cli->BufferSerialInput[1]=='S'){
	    		//read servo reference
	    		cli->rcvNumberString = cli->BufferSerialInput.substring(3,cli->BufferSerialInput.length()-1);
	    		//Setpoint_Servo = rcvNumberString.toDouble();
	    		//Serial.println(Setpoint_Servo);
	    	}
	    	else if(cli->BufferSerialInput[1]=='M'){
	    		cli->rcvNumberString = cli->BufferSerialInput.substring(3,cli->BufferSerialInput.length()-1);
	    		ref->speedVal = cli->rcvNumberString.toDouble();
	    		Serial.println(ref->speedVal);
	    	}
	    }

	    // clear the string:
	    cli->BufferSerialInput = "";
	    cli->FlagBufferInput = false;
}
