// This #include statement was automatically added by the Particle IDE.
#include "mcp2515.h"
#include "can.h"

struct can_frame canMsg1;
struct can_frame canMsg2;
struct can_frame rx;
MCP2515 mcp2515(A5);
int result = -1;
int readin = -1;
char doggo[6] = {"Doggo"};

void setup()
{

    canMsg1.can_id = 0x02;
    canMsg1.can_dlc = 3;
    canMsg1.data[0] = (unsigned char)'C';
    canMsg1.data[1] = (unsigned char)'A';
    canMsg1.data[2] = (unsigned char)'T';

    while (!Serial)
        ;
    Serial.begin(115200);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    Serial.println("------------ Example: Read/Write to CAN ------------");
}

void printError(uint8_t err)
{

    switch (err)
    {
    case 0:
        Serial.print("Error Code: ERROR_OK, ");
        Serial.println(err);
        break;
    case 1:
        Serial.print("Error Code: ERROR_FAIL, ");
        Serial.println(err);
        break;
    case 2:
        Serial.print("Error Code: ERROR_ALLTXBUSY, ");
        Serial.println(err);
        break;
    case 3:
        Serial.print("Error Code: ERROR_FAILINIT, ");
        Serial.println(err);
        break;
    case 4:
        Serial.print("Error Code: ERROR_FAILTX, ");
        Serial.println(err);
        break;
    case 5:
        Serial.print("Error Code: ERROR_NOMSG, ");
        Serial.println(err);
        break;
    default:
        Serial.print("Error Code: UNKNOWN, ");
        Serial.println(err);
        break;
    }
}

void loop()
{
    result = mcp2515.sendMessage(&canMsg1);

    Serial.println("\nMessage has sent...");
    printError(result);

    delay(1000);
    Serial.println("Reading!!!!!!!!!!!1... ");
    readin = mcp2515.readMessage(&rx);
    Serial.print("\nReadin result: ");
    printError(result);
    if (!(readin == MCP2515::ERROR_OK))
    {
        // do nothing
    }
    Serial.println("------- CAN Read ---------\n");
    Serial.println("ID  DLC     DATA\n");
    Serial.print(rx.can_id); // print ID
    Serial.print("   ");
    Serial.print(rx.can_dlc); // print DLC
    Serial.print("   ");

    for (int i = 0; i < rx.can_dlc; i++)
    { // print the data
        Serial.print(rx.data[i]);
        Serial.print(" ");
    }

    printf("\n");
    delay(100);
}
