#include <Arduino.h>

// UART Configuration
#define SERIAL_BAUD 115200 // USB Serial baud rate (for user interface)
#define UART2_BAUD 115200  // UART2 baud rate (matches microcontroller)
#define UART2_RX_PIN 16    // GPIO16 for UART2 RX
#define UART2_TX_PIN 17    // GPIO17 for UART2 TX

// LDPC Protocol Constants
#define LDPC_TAG_0 0xde
#define LDPC_TAG_1 0xad
#define LDPC_TAG_2 0xc0
#define LDPC_TAG_3 0xde
#define MAX_MESSAGE_LENGTH 1024

// System states
enum SystemState
{
  STATE_IDLE,
  STATE_WAITING_FOR_TAG,
  STATE_WAITING_FOR_PARAMS,
  STATE_ENCODING,
  STATE_RECEIVING_RESULT
};

// Input modes
enum InputMode
{
  INPUT_TEXT = 1,
  INPUT_HEX = 2,
  INPUT_HEX_MANUAL = 3
};

SystemState currentState = STATE_IDLE;
InputMode inputMode = INPUT_TEXT;

// LDPC parameters
uint16_t K = 0; // Information bits
uint16_t N = 0; // Codeword bits
uint16_t message_bits = 0;
uint8_t message_buffer[MAX_MESSAGE_LENGTH];
uint8_t encoded_buffer[MAX_MESSAGE_LENGTH * 2]; // Encoded data might be larger
InputMode lastInputMode = INPUT_TEXT;           // Track the last input mode used

void printMenu()
{
  Serial.println("LDPC Encoder Client Menu:");
  Serial.println("1 - Encode text message");
  Serial.println("2 - Encode hex message");
  Serial.println("3 - Encode hex message with manual bit length");
  Serial.println("4 - Check system status");
  Serial.println("5 - Show last encoding results");
  Serial.println("Enter your choice (1-5): ");
}

void printBytes(const uint8_t *data, uint16_t length, bool asHex = true)
{
  if (asHex)
  {
    for (uint16_t i = 0; i < length; i++)
    {
      Serial.printf("%02X", data[i]);
      if ((i + 1) % 16 == 0)
        Serial.println();
      else if ((i + 1) % 4 == 0)
        Serial.print(" ");
    }
    if (length % 16 != 0)
      Serial.println();
  }
  else
  {
    for (uint16_t i = 0; i < length; i++)
    {
      if (data[i] >= 32 && data[i] <= 126)
        Serial.write(data[i]);
      else
        Serial.print('.');
    }
    Serial.println();
  }
}

bool waitForTag()
{
  Serial.println("Waiting for microcontroller tag...");
  uint8_t tagBytes[4] = {0};
  int tagIndex = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) // 5 second timeout
  {
    if (Serial2.available())
    {
      uint8_t receivedByte = Serial2.read();

      if (receivedByte == LDPC_TAG_0 && tagIndex == 0)
      {
        tagBytes[0] = receivedByte;
        tagIndex = 1;
      }
      else if (receivedByte == LDPC_TAG_1 && tagIndex == 1)
      {
        tagBytes[1] = receivedByte;
        tagIndex = 2;
      }
      else if (receivedByte == LDPC_TAG_2 && tagIndex == 2)
      {
        tagBytes[2] = receivedByte;
        tagIndex = 3;
      }
      else if (receivedByte == LDPC_TAG_3 && tagIndex == 3)
      {
        tagBytes[3] = receivedByte;
        Serial.println("Tag received successfully!");
        return true;
      }
      else
      {
        tagIndex = 0; // Reset if sequence breaks
      }
    }
    delay(1);
  }

  Serial.println("Timeout waiting for tag!");
  return false;
}

bool sendMessageLength(uint16_t bits)
{
  uint8_t len_hi = (uint8_t)(bits >> 8);
  uint8_t len_lo = (uint8_t)(bits & 0xFF);

  Serial2.write(len_hi);
  delay(10);
  Serial2.write(len_lo);
  delay(10);

  Serial.printf("Sent message length: %d bits\n", bits);
  return true;
}

bool receiveParameters()
{
  Serial.println("Waiting for K and N parameters...");
  unsigned long startTime = millis();

  while (millis() - startTime < 3000) // 3 second timeout
  {
    if (Serial2.available() >= 4)
    {
      uint8_t k_hi = Serial2.read();
      uint8_t k_lo = Serial2.read();
      uint8_t n_hi = Serial2.read();
      uint8_t n_lo = Serial2.read();

      K = ((uint16_t)k_hi << 8) | (uint16_t)k_lo;
      N = ((uint16_t)n_hi << 8) | (uint16_t)n_lo;

      Serial.printf("Received parameters: K=%d, N=%d\n", K, N);
      return true;
    }
    delay(10);
  }

  Serial.println("Timeout waiting for parameters!");
  return false;
}

bool sendMessageData(const uint8_t *data, uint16_t messageBits, uint16_t calculationBits = 0)
{
  uint16_t K_bytes = (K + 7) / 8;
  uint16_t bitsForCalculation = (calculationBits > 0) ? calculationBits : messageBits;
  uint16_t C = (bitsForCalculation + K - 1) / K; // Number of blocks

  Serial.printf("Sending %d blocks of %d bytes each\n", C, K_bytes);
  Serial.printf("Using %d bits for calculation, sending %d bits of actual data\n", bitsForCalculation, messageBits);

  for (uint16_t block = 0; block < C; block++)
  {
    Serial.printf("Sending block %d/%d...\n", block + 1, C);

    // Send K_bytes for this block
    for (uint16_t i = 0; i < K_bytes; i++)
    {
      uint16_t dataIndex = block * K_bytes + i;
      uint8_t byteToSend = (dataIndex < (messageBits + 7) / 8) ? data[dataIndex] : 0;
      Serial2.write(byteToSend);
      delay(10); // Delay to not overwhelm the MCU
    }

    // Wait for encoded data
    uint16_t N_bytes = (N + 7) / 8;
    Serial.printf("Waiting for %d encoded bytes...\n", N_bytes);

    unsigned long startTime = millis();
    uint16_t receivedBytes = 0;

    while (receivedBytes < N_bytes && (millis() - startTime < 3000))
    {
      if (Serial2.available())
      {
        encoded_buffer[block * N_bytes + receivedBytes] = Serial2.read();
        receivedBytes++;
      }
      delay(1);
    }

    if (receivedBytes < N_bytes)
    {
      Serial.printf("Timeout receiving encoded data for block %d\n", block + 1);
      return false;
    }

    Serial.printf("Received %d encoded bytes for block %d\n", receivedBytes, block + 1);
  }

  return true;
}

uint16_t textToBits(const String &text, uint8_t *buffer)
{
  uint16_t byteCount = min((uint16_t)text.length(), (uint16_t)(MAX_MESSAGE_LENGTH - 1));
  for (uint16_t i = 0; i < byteCount; i++)
  {
    buffer[i] = (uint8_t)text.charAt(i);
  }
  return byteCount * 8; // Convert bytes to bits
}

uint16_t hexToBits(const String &hexStr, uint8_t *buffer)
{
  String cleanHex = hexStr;
  cleanHex.replace(" ", "");
  cleanHex.replace("\n", "");
  cleanHex.replace("\r", "");
  cleanHex.toUpperCase();

  uint16_t byteCount = 0;
  for (uint16_t i = 0; i < cleanHex.length() && i + 1 < cleanHex.length() && byteCount < MAX_MESSAGE_LENGTH; i += 2)
  {
    String hexByte = cleanHex.substring(i, i + 2);
    buffer[byteCount] = (uint8_t)strtol(hexByte.c_str(), NULL, 16);
    byteCount++;
  }
  return byteCount * 8; // Convert bytes to bits
}

void handleEncoding(InputMode mode)
{
  uint16_t manual_message_bits;

  lastInputMode = mode; // Store the input mode for later reference

  if (mode == INPUT_HEX_MANUAL)
  {
    Serial.println("Enter message length: ");

    // Wait for user input
    while (!Serial.available())
    {
      delay(100);
    }

    String lengthInput = Serial.readStringUntil('\n');
    lengthInput.trim();
    manual_message_bits = lengthInput.toInt();
    Serial.printf("Manual message length set to: %d bits\n", manual_message_bits);
  }

  Serial.println("Enter your message:");
  if (mode == INPUT_TEXT)
  {
    Serial.println("(Type your text message and press Enter)");
  }
  else
  {
    Serial.println("(Enter hex bytes, e.g., 'AB CD EF 12' and press Enter)");
  }

  // Wait for user input
  while (!Serial.available())
  {
    delay(100);
  }

  String userInput = Serial.readStringUntil('\n');
  userInput.trim();

  if (userInput.length() == 0)
  {
    Serial.println("No message entered!");
    return;
  }

  Serial.println("Message entered: " + userInput);

  // Convert input to bits
  if (mode == INPUT_TEXT)
  {
    message_bits = textToBits(userInput, message_buffer);
  }
  else
  {
    message_bits = hexToBits(userInput, message_buffer);
  }

  Serial.printf("Message converted to %d bits (%d bytes)\n", message_bits, (message_bits + 7) / 8);

  // Start LDPC encoding process
  Serial.println("\nStarting LDPC encoding process...");

  if (!waitForTag())
  {
    Serial.println("Failed to receive tag from microcontroller!");
    return;
  }

  if (mode == INPUT_HEX_MANUAL)
  {
    if (!sendMessageLength(manual_message_bits))
    {
      Serial.println("Failed to send message length!");
      return;
    }
  }
  else
  {
    if (!sendMessageLength(message_bits))
    {
      Serial.println("Failed to send message length!");
      return;
    }
  }

  if (!receiveParameters())
  {
    Serial.println("Failed to receive LDPC parameters!");
    return;
  }

  uint16_t bitsUsedForCalculation = (mode == INPUT_HEX_MANUAL) ? manual_message_bits : message_bits;

  if (!sendMessageData(message_buffer, message_bits, (mode == INPUT_HEX_MANUAL) ? manual_message_bits : 0))
  {
    Serial.println("Failed to send message data!");
    return;
  }

  Serial.println("\nEncoding completed successfully!");
  Serial.println("=================================");
  Serial.printf("Original message (%d bits, %d bits used for calculation):\n", message_bits, bitsUsedForCalculation);
  printBytes(message_buffer, (message_bits + 7) / 8, mode != INPUT_TEXT); // Display as ASCII for text input, display as hex for hex input
  Serial.printf("\nEncoded data (%d bits per block, %d blocks):\n", N, (bitsUsedForCalculation + K - 1) / K);
  uint16_t totalEncodedBytes = ((bitsUsedForCalculation + K - 1) / K) * ((N + 7) / 8);
  printBytes(encoded_buffer, totalEncodedBytes, true);
  Serial.println();
}

void setup()
{
  // Initialize USB Serial (for user interface)
  Serial.begin(SERIAL_BAUD);

  // Initialize UART2 for microcontroller communication
  Serial2.begin(UART2_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  // Wait for USB Serial to be ready
  while (!Serial)
  {
    delay(10);
  }

  Serial.println("ESP32 LDPC Encoder Client Started");
  Serial.println("==================================");
  Serial.println("Configuration:");
  Serial.printf("USB Serial: %d baud\n", SERIAL_BAUD);
  Serial.printf("UART2: %d baud, RX=GPIO%d, TX=GPIO%d\n", UART2_BAUD, UART2_RX_PIN, UART2_TX_PIN);
  Serial.println();

  printMenu();
}

void loop()
{
  if (Serial.available())
  {
    char choice = Serial.read();

    // Clear any remaining characters in buffer
    while (Serial.available())
    {
      Serial.read();
    }

    Serial.println(); // New line after choice

    switch (choice)
    {
    case '1':
      Serial.println("Text encoding mode selected");
      handleEncoding(INPUT_TEXT);
      break;
    case '2':
      Serial.println("Hex encoding mode selected");
      handleEncoding(INPUT_HEX);
      break;
    case '3':
      Serial.println("Hex encoding mode with manual bit length selected");
      handleEncoding(INPUT_HEX_MANUAL);
      break;
    case '4':
      Serial.println("System Status:");
      Serial.printf("Current state: %d\n", currentState);
      Serial.printf("Last K: %d, Last N: %d\n", K, N);
      Serial.printf("Last message bits: %d\n", message_bits);
      break;
    case '5':
      if (K > 0 && N > 0 && message_bits > 0)
      {
        Serial.println("Last encoding results:");
        Serial.printf("K=%d, N=%d, Message bits=%d\n", K, N, message_bits);
        Serial.println("Original message:");
        printBytes(message_buffer, (message_bits + 7) / 8, lastInputMode == INPUT_HEX); // Display as ASCII for text input, display as hex for hex input
        Serial.println("Encoded data:");
        uint16_t totalEncodedBytes = ((message_bits + K - 1) / K) * ((N + 7) / 8);
        printBytes(encoded_buffer, totalEncodedBytes, true);
      }
      else
      {
        Serial.println("No encoding results available yet.");
      }
      break;
    default:
      Serial.println("Invalid choice!");
      break;
    }

    Serial.println();
    printMenu();
  }

  delay(10);
}