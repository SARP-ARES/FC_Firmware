


void readPacket(uint32_t address, FlightPacket& pkt) {
    read(address, reinterpret_cast<uint8_t*>(&pkt), sizeof(FlightPacket));
}


void printCSVHeader() {
    pc.printf("timestamp_ms,latitude,longitude,altitude,velocity,temp_c,status,id\n");
}

void printPacketAsCSV(const FlightPacket& pkt) {
    pc.printf("%lu,%.7f,%.7f,%.2f,%.2f,%d,%u,%s\n",
        pkt.timestamp_ms,
        pkt.latitude,
        pkt.longitude,
        pkt.altitude,
        pkt.velocity,
        pkt.temp_c,
        pkt.status,
        pkt.id);
}


void dumpAllPackets(flash& fc, uint32_t numPackets) {
    FlightPacket pkt;
    printCSVHeader();
    for (uint32_t i = 0; i < numPackets; ++i) {
        fc.readPacket(i * sizeof(FlightPacket), pkt);
        printPacketAsCSV(pkt);
    }
}
