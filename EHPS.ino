#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);

static uint32_t const ID_WAKEUP   = 0x201;
static uint32_t const ID_STEER    = 0x082;
static uint32_t const ID_PUMP_RSP = 0x240;

const int TX_INTERVAL_MS   = 20;
const int DIAG_INTERVAL_MS = 2000;

unsigned long lastTxTime   = 0;
unsigned long lastDiagTime = 0;

uint16_t tx1_attempts = 0, tx1_errors = 0;
uint16_t tx2_attempts = 0, tx2_errors = 0;
uint16_t rx_total     = 0;
uint16_t rx_pump      = 0;
uint16_t rx_other     = 0;

struct can_frame wakeupFrame;
struct can_frame steerFrame;

void setup() {
  Serial.begin(115200);
  SPI.begin(); // Manually start SPI
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); 
  while (!Serial) delay(10);

  Serial.println("=== CAN PUMP DIAGNOSTIC (MCP2515) ===");
  Serial.println("Initializing MCP2515 at 250k, 8MHz crystal...");

  mcp2515.reset();
  delay(500);

  /**
  Try these three initialization settings one by one. They are the most common "near-miss" baud rates for industrial pumps:

    Try 250kbps with "16MHz" setting (even though your crystal is 8MHz):
    mcp2515.setBitrate(CAN_250KBPS, MCP_16MHZ);
    (Some libraries auto-scale the math differently; this is a common "trick" fix).
    Try 125kbps:
    mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    (Many hydraulic controllers actually default to 125k for long-wire stability).
    Try 500kbps again with the now-stable SPI:
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  
  */

  MCP2515::ERROR err;
  int attempts = 0;
  do {
    err = mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    attempts++;
    Serial.print("setBitrate attempt ");
    Serial.print(attempts);
    Serial.print(" result: ");
    Serial.println(err);
    delay(100);
  } while (err != MCP2515::ERROR_OK && attempts < 10);

  if (err != MCP2515::ERROR_OK) {
    Serial.println("[FAIL] setBitrate() failed after 10 attempts.");
    Serial.println("       Check: CS on D10, MOSI D11, MISO D12, SCK D13, VCC 5V");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(200);
      digitalWrite(LED_BUILTIN, LOW);  delay(200);
    }
  }

  // ← THIS WAS MISSING
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("[FAIL] setNormalMode() failed.");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(200);
      digitalWrite(LED_BUILTIN, LOW);  delay(200);
    }
  }

  wakeupFrame.can_id  = ID_WAKEUP;
  wakeupFrame.can_dlc = 8;
  wakeupFrame.data[0] = 0x05;
  wakeupFrame.data[1] = 0xD0;
  wakeupFrame.data[2] = 0x00;
  wakeupFrame.data[3] = 0x00;
  wakeupFrame.data[4] = 0x00;
  wakeupFrame.data[5] = 0x00;
  wakeupFrame.data[6] = 0x00;
  wakeupFrame.data[7] = 0x00;

  steerFrame.can_id  = ID_STEER;
  steerFrame.can_dlc = 8;
  steerFrame.data[0] = 0x00;
  steerFrame.data[1] = 0x32;
  steerFrame.data[2] = 0x00;
  steerFrame.data[3] = 0x00;
  steerFrame.data[4] = 0x00;
  steerFrame.data[5] = 0x00;
  steerFrame.data[6] = 0x10;
  steerFrame.data[7] = 0x00;

  Serial.println("[OK]   MCP2515 initialized — starting 50Hz transmission immediately.");
  Serial.println("---");
  Serial.println("Columns: TX1_sent/err | TX2_sent/err | RX_total | RX_0x240 | RX_other | notes");
  Serial.println("---");
}

void drainRxBuffer() {
  struct can_frame incoming;
  while (mcp2515.readMessage(&incoming) == MCP2515::ERROR_OK) {
    rx_total++;
    if (incoming.can_id == ID_PUMP_RSP) {
      rx_pump++;
      Serial.print("[RX 0x240] ");
      for (int i = 0; i < incoming.can_dlc; i++) {
        if (incoming.data[i] < 0x10) Serial.print("0");
        Serial.print(incoming.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      rx_other++;
      Serial.print("[RX 0x");
      Serial.print(incoming.can_id, HEX);
      Serial.print("] ");
      for (int i = 0; i < incoming.can_dlc; i++) {
        if (incoming.data[i] < 0x10) Serial.print(incoming.data[i], HEX);
        Serial.print(incoming.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}

void printDiagnostics() {
  Serial.print("TX 0x201: "); Serial.print(tx1_attempts); Serial.print(" sent, ");
  Serial.print(tx1_errors);   Serial.print(" err | ");
  Serial.print("TX 0x082: "); Serial.print(tx2_attempts); Serial.print(" sent, ");
  Serial.print(tx2_errors);   Serial.print(" err | ");
  Serial.print("RX total: "); Serial.print(rx_total);  Serial.print(" | ");
  Serial.print("RX 0x240: "); Serial.print(rx_pump);   Serial.print(" | ");
  Serial.print("RX other: "); Serial.print(rx_other);  Serial.print(" | ");

  if (tx1_errors > 10 || tx2_errors > 10) {
    Serial.print("[ISSUE] High TX errors — check wiring and termination.");
  } else if (rx_total == 0) {
    Serial.print("[ISSUE] No frames received — check CANH/CANL.");
  } else if (rx_total > 0 && rx_pump == 0) {
    Serial.print("[ISSUE] Bus traffic but no 0x240 — check pump 12V and ground.");
  } else if (rx_pump > 0) {
    Serial.print("[OK] Pump responding on 0x240!");
  } else {
    Serial.print("[OK] Transmitting cleanly — waiting for pump response.");
  }

  Serial.println();

  tx1_attempts = tx1_errors = 0;
  tx2_attempts = tx2_errors = 0;
  rx_total = rx_pump = rx_other = 0;
}

void loop() {
  unsigned long now = millis();

  uint8_t errFlags = mcp2515.getErrorFlags();
  if (errFlags) {
    Serial.print("[MCP ERROR FLAGS] 0x");
    Serial.println(errFlags, HEX);
    if (errFlags & MCP2515::EFLG_TXBO)  Serial.println("  -> TX Bus Off");
    if (errFlags & MCP2515::EFLG_TXEP)  Serial.println("  -> TX Error Passive");
    if (errFlags & MCP2515::EFLG_RXEP)  Serial.println("  -> RX Error Passive");
    if (errFlags & MCP2515::EFLG_TXWAR) Serial.println("  -> TX Warning");
    if (errFlags & MCP2515::EFLG_RXWAR) Serial.println("  -> RX Warning");

    if (errFlags & MCP2515::EFLG_TXBO) {
      Serial.println("  -> Recovering from bus-off...");
      mcp2515.reset();
      delay(100);
      mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
      mcp2515.setNormalMode();
    }
  }

  drainRxBuffer();

  if (now - lastTxTime >= TX_INTERVAL_MS) {
    lastTxTime = now;

    tx1_attempts++;
    if (mcp2515.sendMessage(&wakeupFrame) != MCP2515::ERROR_OK) tx1_errors++;
    delay(5);
    tx2_attempts++;
    if (mcp2515.sendMessage(&steerFrame) != MCP2515::ERROR_OK) tx2_errors++;
  }

  if (now - lastDiagTime >= DIAG_INTERVAL_MS) {
    lastDiagTime = now;
    printDiagnostics();
  }
}
