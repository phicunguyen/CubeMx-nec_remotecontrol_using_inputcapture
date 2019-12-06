# CubeMx-nec_remotecontrol_using_inputcapture
This code is using the input capture mode on stm32 to detect the nec remote control. (TSOP39238 IR Receive).

It uses the timer2 input capture direct mode to log the time between two falling edge. If the expecting value is in the range then move to next state. 

    void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {
       if (htim->Instance == TIM2) {  
           //capture the time between two falling edges.
           icVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        
           //should be 9ms+4.5ms = 13.5ms
           //The IR_DETTA return true if the start frame is between +/-4% of 13.5ms
           if (IR_DELTA(icVal, IR_BIT__START_FRAME)) {
              irBitCnt=2;  //two pulse added 
              irCode=0;
           } 
           else if (irBitCnt >= 0x02) {
               //1.25ms +/-4%
               irLow = IR_DELTA(icVal, IR_BIT__LOW);
               //2.25ms +/-4%
               irHigh = IR_DELTA(icVal, IR_BIT__HIGH);
               if (irLow || irHigh) {
                   irBitCnt++;
                   irCode <<= 1;
                   if (irHigh) {
                       irCode |= 1;
                }   
           } else {
                //the ir data is not valid
                //reset the state
                irBitCnt=0;
                irCode = 0;
            }

            //Detect the ir command and data
            if (irBitCnt >= 32) {
                irNewCode = irCode;
                irBitCnt=0;
                irCode = 0;
                //should copy the irCode to the user.
            }
        }
        //reset the counter.
        __HAL_TIM_SET_COUNTER(htim,0);        
    }
    
