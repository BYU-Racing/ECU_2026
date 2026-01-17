#include <FlexCAN_T4.h>

template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize = RX_SIZE_16, FLEXCAN_TXQUEUE_TABLE _txSize = TX_SIZE_16>
class CanController {
    private:
        FlexCAN_T4<_bus, _rxSize, _txSize> can;

    public:
        CanController(uint32_t baud_rate) {
            can.begin();
            can.setBaudRate(baud_rate);
        }

        CAN_message_t sniff() {

        }

        void writeLight(bool status) {
            write()
        }
};

uint16_t parsePedalMessage(CAN_message_t msg) {
    return msg.buf[0] << 8 | msg.buf[1];
}

constexpr int PEDAL_MESSAGE = 2;

void setup() {
    CanController controller(250000);

    while (true) {
        CAN_message_t msg = controller.sniff();
        switch (msg.id) {
            case PEDAL_MESSAGE:
                uint16_t pedal = parsePedalMessage(msg);
                handlePedalMessage(pedal);
                break;
        }
    }
}