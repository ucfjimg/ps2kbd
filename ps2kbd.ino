/**
 * Understands how to parse a stream of incoming key codes and manage press/release and shift
 */

class PS2Keyboard;
class PS2KeyHandler
{
public:
  PS2KeyHandler(PS2Keyboard &keyboard);
  void onFrame(unsigned data, bool error);
  
private:
  struct Key {
    char regular;
    char shifted;
  };

  enum {
    BREAK_CODE = 0xf0,
    LSHIFT_CODE = 0x12,
    RSHIFT_CODE = 0x59,
    CAPSLOCK_CODE = 0x58,

    LSHIFT_MASK = 0x01,
    RSHIFT_MASK = 0x02,
    SHIFT_MASK = (LSHIFT_MASK | RSHIFT_MASK),
    CAPSLOCK_MASK = 0x04,
    N_KEYS = 128
  };
  
  static Key keys_[N_KEYS];     // keyboard lookup 

  PS2Keyboard &keyboard_;       // for sending commands
  unsigned shiftState_;         // current shift state
  bool nextIsRelease_;          // next scan code is a key UP code

  static unsigned shiftToMask(unsigned code);
  void down(unsigned code);
  void up(unsigned code);
};

/**
 * Handles protocol decoding for the PS/2 keyboard.
 */
class PS2Keyboard
{
public:
  PS2Keyboard(int clockIn, int dataIn, int clockOut, int dataOut);
  
  void init();
  void run();  

  enum {
    ScrollLock = 0x01,
    NumLock = 0x02,
    CapsLock = 0x04
  } LEDs;

  void turnOnLEDs(unsigned mask);
  void turnOffLEDs(unsigned mask);

  bool enqueueCommand(const uint8_t *cmd, int len);

private:

  // What state are we in receiving a frame
  enum FrameState
  {
    Idle,
    Data,
    ParityBit,
    StopBit,
  };

  
  // Why did we fail to receive a proper frame
  enum FrameError
  {
    None,
    StartBitError,
    ParityError,
    StopBitError,
  };

  // some command command bytes
  enum CommandByte
  {
    SET_LEDS = 0xED,
    COMMAND_ACK = 0xFA,
    COMMAND_NAK = 0xFE,  
  };

  // scan code to ASCII conversion
  PS2KeyHandler keyHandler_;

  // current state
  FrameState frameState_;

  // Pin definitions
  int clockIn_;
  int dataIn_;
  int clockOut_;
  int dataOut_;

  bool deviceClockHigh_;      // Our understanding of the current clock state
  int bitCount_;              // Countdown of bits received in data portion of frame
  FrameError frameError_;     // If there's a receive error, why
  unsigned frameData_;        // Buffer for incoming frame byte

  enum { MAX_SEND_Q = 8 };
  
  uint8_t sendQueue_[MAX_SEND_Q]; // Queue for host->device data
  int sendQHead_;                 // Start of queue 
  int sendQTail_;                 // End of queue
  bool wantCommandAck_;           // Waiting for a command response

  unsigned ledState_;             // lights on/off?

  void sendQueueByte();
  void commandAck(unsigned code, bool error);
  void resetReceiveState();
  void receiveBit(int value);
  static int parityBit(unsigned data);
};

/**
 * Constructor.
 */
PS2KeyHandler::PS2KeyHandler(PS2Keyboard &keyboard)
  : keyboard_(keyboard)
  , shiftState_(0)
  , nextIsRelease_(false)
{
}

void PS2KeyHandler::onFrame(unsigned data, bool error)
{
  if (error) {
    nextIsRelease_ = false;
    return;
  }

  if (data == BREAK_CODE) {
    nextIsRelease_ = true;
    return;
  }

  if (nextIsRelease_) {
    nextIsRelease_ = false;
    up(data);
    return;
  }

  down(data);
}

/**
 * Convert `code' to a shift mask if it's a shift key, else return 0.
 */
unsigned PS2KeyHandler::shiftToMask(unsigned code)
{
  switch (code) {
  case LSHIFT_CODE: return LSHIFT_MASK;
  case RSHIFT_CODE: return RSHIFT_MASK;
  }

  return 0;
}

/** 
 * Handle a key press event. 
 */
void PS2KeyHandler::down(unsigned code)
{
  unsigned shift = shiftToMask(code);
  if (shift) {
    shiftState_ |= shift;
    return;
  }

  if (code == CAPSLOCK_CODE) {
    shiftState_ ^= CAPSLOCK_MASK;
    if ((shiftState_ & CAPSLOCK_MASK) != 0) {
      keyboard_.turnOnLEDs(PS2Keyboard::CapsLock);
    } else {
      keyboard_.turnOffLEDs(PS2Keyboard::CapsLock);
    }
    return;
  }

  if (code >= N_KEYS) {
    return;
  }

  const Key &key = keys_[code];
  char unshifted = key.regular;
  char toPrint = '\0';
  if (unshifted >= 'a' && unshifted <= 'z') {
    bool shifted = (shiftState_ & CAPSLOCK_MASK) != 0;
    if ((shiftState_ & SHIFT_MASK) != 0) {
      shifted = !shifted;
    }
    toPrint = shifted ? key.shifted : key.regular;
  } else {
    toPrint = ((shiftState_ & SHIFT_MASK) != 0) ? key.shifted : key.regular;
  }

  if (toPrint) {
    Serial.print(toPrint);
  }
}

/**
 * Handle a key release event.
 */
void PS2KeyHandler::up(unsigned code)
{
  unsigned shift = shiftToMask(code);
  if (shift) {
    shiftState_ &= ~shift;
  }
}

/**
 * Constructor
 */
PS2Keyboard::PS2Keyboard(int clockIn, int dataIn, int clockOut, int dataOut)
  : keyHandler_(*this)
  , clockIn_(clockIn)
  , dataIn_(dataIn)
  , clockOut_(clockOut)
  , dataOut_(dataOut)
  , deviceClockHigh_(true)
  , bitCount_(0)
  , frameError_(None)
  , frameState_(Idle)
  , frameData_(0)
  , sendQHead_(0)
  , sendQTail_(0)
  , wantCommandAck_(false)
  , ledState_(0)
{
}

void PS2Keyboard::turnOnLEDs(unsigned mask)
{
  ledState_ |= mask;
  uint8_t command[] = { SET_LEDS, ledState_ };
  enqueueCommand(command, 2); 
}

void PS2Keyboard::turnOffLEDs(unsigned mask)
{
  ledState_ &= ~mask;
  uint8_t command[] = { SET_LEDS, ledState_ };
  enqueueCommand(command, 2); 
}

bool PS2Keyboard::enqueueCommand(const uint8_t *cmd, int len)
{
  // This could be a circular queue but the channel in this direction
  // never gets enough concurrent traffic to justify it.
  if (sendQTail_ + len > MAX_SEND_Q) {
    return false;
  }

  while (len--) {
    sendQueue_[sendQTail_++] = *cmd++;
  }

  if (sendQHead_ == 0) {
    sendQueueByte();
  }

  return true;
}

/**
 * Initialize the keyboard interface.
 */
void PS2Keyboard::init()
{
  // set up pins
  pinMode(clockIn_, INPUT);
  pinMode(dataIn_, INPUT);
  pinMode(clockOut_, OUTPUT);
  pinMode(dataOut_, OUTPUT);

  // float clock and data outputs 
  digitalWrite(clockOut_, LOW);
  digitalWrite(dataOut_, LOW);
}

/**
 * Called periodically. Follow the device's clock and receive incoming frame
 * bits.
 */
void PS2Keyboard::run()
{
  // Track clock input from device. Data is read on the falling edge of the clock.
  if (deviceClockHigh_ && digitalRead(clockIn_) == LOW) {
    receiveBit(digitalRead(dataIn_) == HIGH ? 1 : 0);
    deviceClockHigh_ = false;
    return;
  }

  if (!deviceClockHigh_ && digitalRead(clockIn_) == HIGH) {
    deviceClockHigh_ = true;
  }
}

/**
 * Send the next byte in the queue
 */
void PS2Keyboard::sendQueueByte()
{
  wantCommandAck_ = false;
  
  if (sendQHead_ == sendQTail_) {
    sendQHead_ = 0;
    sendQTail_ = 0;
    return;  
  }

  // this needs to be done fast and nothing else can happen while it's going on, so send
  // the entire byte here at once rather than using state machine mechanics.

  // NB throughout here, the data and clock outputs are driven by transistors pulling the 
  // corresponding line LOW when the pin is HIGH. Thus all the writes here are the opposite
  // of what would be intuitive.

  uint8_t frame = sendQueue_[sendQHead_++];
  unsigned parity = parityBit(frame);

  noInterrupts();

  // Get device's attention
  digitalWrite(clockOut_, HIGH);
  delayMicroseconds(100);
  
  // Send start bit
  digitalWrite(dataOut_, HIGH);
  digitalWrite(clockOut_, LOW);
  
  for (int i = 0; i < 8; i++) {
    while (digitalRead(clockIn_) != LOW);

    if (frame & 0x01) {
      digitalWrite(dataOut_, LOW);
     } else {
      digitalWrite(dataOut_, HIGH);
    }
    frame >>= 1;

    while (digitalRead(clockIn_) != HIGH);    
  }

  // parity bit
  while (digitalRead(clockIn_) != LOW);
  digitalWrite(dataOut_, parity ? LOW : HIGH);
  while (digitalRead(clockIn_) != HIGH);    
  while (digitalRead(clockIn_) != LOW);
  // stop bit
  digitalWrite(dataOut_, LOW); 
  while (digitalRead(clockIn_) != HIGH);    
  while (digitalRead(clockIn_) != LOW);

  interrupts();  

  wantCommandAck_ = true;
}

/**
 * Handle a command ack or nak
 */
void PS2Keyboard::commandAck(unsigned code, bool error)
{
  if (error) {
    // just flush queue
    Serial.print("Error processing comment queue\n");
    wantCommandAck_ = false;
    sendQHead_ = 0;
    sendQTail_ = 0;
    return;
  }

  switch (code) {
  case COMMAND_ACK: /* ACK */
    sendQueueByte();
    break;

  case COMMAND_NAK: /* NAK, resend */
    sendQHead_--;
    sendQueueByte();
    break;

  default:
    Serial.print("Keyboard replied ");
    Serial.print(code, HEX);
    Serial.print(" to command.\n");
    sendQHead_ = 0;
    sendQTail_ = 0;
    break;
  }
}

/**
 * Set up the receiver FSM for the next frame.
 */
void PS2Keyboard::resetReceiveState()
{
  frameState_ = Idle;
  frameData_ = 0;
  frameError_ = None;
  bitCount_ = 0;
}

/**
 * State machine to handle receiving one bit of a frame.
 */
void PS2Keyboard::receiveBit(int value)
{
  switch (frameState_) {
  case Idle:
    // The start bit is expected to be zero.
    if (value != 0) {
      frameError_ = StartBitError;
    }
    frameState_ = Data;
    bitCount_ = 0;
    frameData_ = 0;
    break;
    
  case Data:
    // Receive data bits. Data comes in least significant bit first. 
    frameData_ |= (value << bitCount_);
    bitCount_++;
    if (bitCount_ == 8) {
      frameState_ = ParityBit;
    }
    break;

  case ParityBit:
    // Check the parity
    if (parityBit(frameData_) != value && frameError_ == None) {
      frameError_ = ParityError;
    }
    frameState_ = StopBit;
    break;

  case StopBit:
    // The stop bit is expected to be one.
    if (value != 1 && frameError_ == None) {
      frameError_ = StopBitError;
    }

    if (frameError_) {
      Serial.print("Frame error ");
      Serial.print(frameError_);
      Serial.print("\n");
    }

    if (wantCommandAck_) {
      commandAck(frameData_, frameError_ != None);
    } else {
      keyHandler_.onFrame(frameData_, frameError_ != None);
    }
    resetReceiveState();
    break;
  }
}

/**
 * Return the expected (odd) parity bit for the given data byte.
 */
int PS2Keyboard::parityBit(unsigned data)
{
  // counts the number of one bits in `data'
  int ones = 0;
  while (data) {
    ones++;
    data &= (data-1);
  }

  // if there are an odd number of one bits, then the parity bit
  // should be zero.
  return (ones & 0x01) ? 0 : 1;
}

/*****************************************************************
 * Default pin hookups
 */
const int CLOCK_IN = 9;
const int DATA_IN = 8;
const int CLOCK_OUT = 5;
const int DATA_OUT = 4;

// build and run the device
PS2Keyboard kbd(CLOCK_IN, DATA_IN, CLOCK_OUT, DATA_OUT);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  for (int i = 0; i < 200; i++) {
    Serial.print("\n");
  }
  Serial.print("Type:\n\n");

  kbd.init();
}

void loop()
{
  kbd.run();
}

PS2KeyHandler::Key PS2KeyHandler::keys_[128] {
  { '\0', '\0' },  // 0x00
  { '\0', '\0' },  // 0x01
  { '\0', '\0' },  // 0x02
  { '\0', '\0' },  // 0x03
  { '\0', '\0' },  // 0x04
  { '\0', '\0' },  // 0x05
  { '\0', '\0' },  // 0x06
  { '\0', '\0' },  // 0x07
  { '\0', '\0' },  // 0x08
  { '\0', '\0' },  // 0x09
  { '\0', '\0' },  // 0x0a
  { '\0', '\0' },  // 0x0b
  { '\0', '\0' },  // 0x0c
  { '\t', '\t' },  // 0x0d
  { '`',  '~' },   // 0x0e
  { '\0', '\0' },  // 0x0f

  { '\0', '\0' },  // 0x10
  { '\0', '\0' },  // 0x11
  { '\0', '\0' },  // 0x12
  { '\0', '\0' },  // 0x13
  { '\0', '\0' },  // 0x14
  { 'q',  'Q'  },  // 0x15
  { '1',  '!'  },  // 0x16
  { '\0', '\0' },  // 0x17
  { '\0', '\0' },  // 0x18
  { '\0', '\0' },  // 0x19
  { 'z',  'Z'  },  // 0x1a
  { 's',  'S'  },  // 0x1b
  { 'a',  'A'  },  // 0x1c
  { 'w',  'W'  },  // 0x1d
  { '2',  '@'  },  // 0x1e
  { '\0', '\0' },  // 0x1f

  { '\0', '\0' },  // 0x20
  { 'c',  'C'  },  // 0x21
  { 'x',  'X'  },  // 0x22
  { 'd',  'D'  },  // 0x23
  { 'e',  'E'  },  // 0x24
  { '4',  '$'  },  // 0x25
  { '3',  '#'  },  // 0x26
  { '\0', '\0' },  // 0x27
  { '\0', '\0' },  // 0x28
  { ' ',  ' '  },  // 0x29
  { 'v',  'V'  },  // 0x2a
  { 'f',  'F'  },  // 0x2b
  { 't',  'T'  },  // 0x2c
  { 'r',  'R'  },  // 0x2d
  { '5',  '\0' },  // 0x2e
  { '\0', '\0' },  // 0x2f

  { '\0', '\0' },  // 0x30
  { 'n' , 'N'  },  // 0x31
  { 'b',  'B'  },  // 0x32
  { 'h',  'H'  },  // 0x33
  { 'g',  'G'  },  // 0x34
  { 'y',  'Y'  },  // 0x35
  { '6',  '^'  },  // 0x36
  { '\0', '\0' },  // 0x37
  { '\0', '\0' },  // 0x38
  { '\0', '\0' },  // 0x39
  { 'm',  'M'  },  // 0x3a
  { 'j' , 'J'  },  // 0x3b
  { 'u',  'U'  },  // 0x3c
  { '7',  '&'  },  // 0x3d
  { '8',  '*'  },  // 0x3e
  { '\0', '\0' },  // 0x3f

  { '\0', '\0' },  // 0x40
  { ',',  '<'  },  // 0x41
  { 'k',  'K'  },  // 0x42
  { 'i',  'I'  },  // 0x43
  { 'o',  'O'  },  // 0x44
  { '0',  ')'  },  // 0x45
  { '9',  '('  },  // 0x46
  { '\0', '\0' },  // 0x47
  { '\0', '\0' },  // 0x48
  { '.',  '>'  },  // 0x49
  { '/',  '?'  },  // 0x4a
  { 'l',  'L'  },  // 0x4b
  { ';',  ':'  },  // 0x4c
  { 'p',  'P'  },  // 0x4d
  { '-',  '_'  },  // 0x4e
  { '\0', '\0' },  // 0x4f

  { '\0', '\0' },  // 0x50
  { '\0', '\0' },  // 0x51
  { '\'', '"'  },  // 0x52
  { '\0', '\0' },  // 0x53
  { '[',  '{'  },  // 0x54
  { '=',  '+'  },  // 0x55
  { '\0', '\0' },  // 0x56
  { '\0', '\0' },  // 0x57
  { '\0', '\0' },  // 0x58
  { '\0', '\0' },  // 0x59
  { '\n', '\n' },  // 0x5a
  { ']',  '}'  },  // 0x5b
  { '\0', '\0' },  // 0x5c
  { '\\', '|'  },  // 0x5d
  { '\0', '\0' },  // 0x5e
  { '\0', '\0' },  // 0x5f

  { '\0', '\0' },  // 0x60
  { '\0', '\0' },  // 0x61
  { '\0', '\0' },  // 0x62
  { '\0', '\0' },  // 0x63
  { '\0', '\0' },  // 0x64
  { '\0', '\0' },  // 0x65
  {   8,    8  },  // 0x66
  { '\0', '\0' },  // 0x67
  { '\0', '\0' },  // 0x68
  { '\0', '\0' },  // 0x69
  { '\0', '\0' },  // 0x6a
  { '\0', '\0' },  // 0x6b
  { '\0', '\0' },  // 0x6c
  { '\0', '\0' },  // 0x6d
  { '\0', '\0' },  // 0x6e
  { '\0', '\0' },  // 0x6f

  { '\0', '\0' },  // 0x70
  { '\0', '\0' },  // 0x71
  { '\0', '\0' },  // 0x72
  { '\0', '\0' },  // 0x73
  { '\0', '\0' },  // 0x74
  { '\0', '\0' },  // 0x75
  { '\0', '\0' },  // 0x76
  { '\0', '\0' },  // 0x77
  { '\0', '\0' },  // 0x78
  { '\0', '\0' },  // 0x79
  { '\0', '\0' },  // 0x7a
  { '\0', '\0' },  // 0x7b
  { '\0', '\0' },  // 0x7c
  { '\0', '\0' },  // 0x7d
  { '\0', '\0' },  // 0x7e
  { '\0', '\0' },  // 0x7f
};
