/*************************************************************************************************************
The presets of 220, 140, 70, 40 may be different based on the hardware you have used. For our case the max 
analog signal that we are sending to our motor is 220.
**************************************************************************************************************/

// PIN Configuration
const int EnableL = 10;
const int HighL = 8;       // RIGHT SIDE MOTOR
const int LowL = 9;

const int EnableR = 5;
const int HighR = 6;       //LEFT SIDE MOTOR
const int LowR = 7;

const int D0 = 0;       //Raspberry pin 21    LSB
const int D1 = 2;       //Raspberry pin 22
const int D2 = 3;       //Raspberry pin 23
const int D3 = 4;       //Raspberry pin 24    MSB

// stores the input from the digital pins, data stores the binary serial data in decimal form
int a,b,c,d,data;


void setup() 
{
	pinMode(EnableL, OUTPUT);      
	pinMode(HighL, OUTPUT);
	pinMode(LowL, OUTPUT);


	pinMode(EnableR, OUTPUT);
	pinMode(HighR, OUTPUT);
	pinMode(LowR, OUTPUT);

	pinMode(D0, INPUT_PULLUP);
	pinMode(D1, INPUT_PULLUP);
	pinMode(D2, INPUT_PULLUP);
	pinMode(D3, INPUT_PULLUP);
}

void Data()
{
	a = digitalRead(D0);
	b = digitalRead(D1);
	c = digitalRead(D2);
	d = digitalRead(D3);
	
	// Convert the data to decimal by reading the PIN values
	data = 8*d + 4*c + 2*b + a;
}

void Forward()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,220);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,220);
}

void Backward()
{
	digitalWrite(HighL, LOW);
	digitalWrite(LowL, HIGH);
	analogWrite(EnableL,255);

	digitalWrite(HighR, LOW);
	digitalWrite(LowR, HIGH);
	analogWrite(EnableR,255);
}

void Stop()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,0);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,0);
}

void Left1()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,140);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,220);
}

void Left2()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,70);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,220);
}


void Left3()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,40);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,220);
}

void Right1()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,220);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,140);  
}

void Right2()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,220);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,70);   
}

void Right3()
{
	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL,220);

	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableR,40);   
}


void UTurn()
{
	analogWrite(EnableL, 0);  // Stop for stability for 400ms
	analogWrite(EnableR, 0);
	delay(400);

	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);  //forward
	delay(1500);

	analogWrite(EnableL, 0);
	analogWrite(EnableR, 0);
	delay(400);

	digitalWrite(HighL, LOW);  // Left side motor to backward direction
	digitalWrite(LowL, HIGH);
	digitalWrite(HighR, HIGH);  // Right side motor to forward direction
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(800);

	analogWrite(EnableL, 0);  // Stop for stability for 400ms
	analogWrite(EnableR, 0);
	delay(400);

	digitalWrite(HighL, HIGH);
	digitalWrite(LowL, LOW);
	digitalWrite(HighR, HIGH);  // Forward
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(1000);

	analogWrite(EnableL, 0); // Stop for stability for 400ms
	analogWrite(EnableR, 0);
	delay(400);

	digitalWrite(HighL, LOW);
	digitalWrite(LowL, HIGH);
	digitalWrite(HighR, HIGH); //Move left
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(900);


	analogWrite(EnableL, 0);  // Stop for stability for 2500ms
	analogWrite(EnableR, 0);
	delay(2500);

	digitalWrite(HighL, HIGH);  // gradually increase the speed
	digitalWrite(LowL, LOW);
	digitalWrite(HighR, HIGH);
	digitalWrite(LowL, LOW);
	analogWrite(EnableL, 150);
	analogWrite(EnableR, 150);
	delay(300);
}


void Object()
{
	analogWrite(EnableL, 0);
	analogWrite(EnableR, 0);  // stop
	delay(1000);

	digitalWrite(HighL, LOW);  // left
	digitalWrite(LowL, HIGH);
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);  
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(500);

	analogWrite(EnableL, 0);  //stop
	analogWrite(EnableR, 0);
	delay(200);

	digitalWrite(HighL, HIGH);  // forward
	digitalWrite(LowL, LOW);           
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(1400);

	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);
	delay(200);

	digitalWrite(HighL, HIGH);  // right
	digitalWrite(LowL, LOW);
	digitalWrite(HighR, LOW);         
	digitalWrite(LowR, HIGH);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(500);

	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);
	delay(1000);

	digitalWrite(HighL, HIGH);  // forward
	digitalWrite(LowL, LOW);           
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(1000);
	Lane_Change();

}

void Lane_Change()
{
	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);            
	delay(1000);

	digitalWrite(HighL, HIGH); // right
	digitalWrite(LowL, LOW);
	digitalWrite(HighR, LOW);
	digitalWrite(LowR, HIGH);        
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(500);

	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);           
	delay(200);

	digitalWrite(HighL, HIGH);  // forward
	digitalWrite(LowL, LOW);           
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(1200);

	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);
	delay(200);

	digitalWrite(HighL, LOW);  // left
	digitalWrite(LowL, HIGH);
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(500);

	analogWrite(EnableL, 0);  // stop
	analogWrite(EnableR, 0);
	delay(1000);

	digitalWrite(HighL, HIGH);  // forward
	digitalWrite(LowL, LOW);
	digitalWrite(HighR, HIGH);
	digitalWrite(LowR, LOW);
	analogWrite(EnableL, 220);
	analogWrite(EnableR, 220);
	delay(1000);
}

void loop() 
{
	// fetch thedata from the digital pins on which Raspberry Pi is writing 0/1
	Data(); 

	if(data==0)
	{
		Forward();
	}
	else if(data == 1)
	{
		Right1();
	}
	else if(data == 2)
	{
		Right2();
	}
	else if(data == 3)
	{
		Right3();
	}
	else if(data == 4)
	{
		Left1();
	}
	else if(data == 5)
	{
		Left2();
	}
	else if(data == 6)
	{
		Left3();
	}
	else if (data == 7)
	{
		UTurn();
	} 
	else if (data == 8)
	{
		analogWrite(EnableL, 0);
		analogWrite(EnableR, 0);
		delay(4000);                      // stop sign detected, stop for 4 seconds then gradually increase the speed

		analogWrite(EnableL, 150);
		analogWrite(EnableR, 150);
		delay(2000);
	}
	else if (data == 9)
	{
		Object();
	}
	else if (data>=10)
	{
		Stop();
	}
}
