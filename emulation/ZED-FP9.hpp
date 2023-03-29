#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <Arduino.h>

namespace emulation
{

#define CAT_FIELDS(MSB, LSB) ((MSB << 8) + LSB)

template<typename T, size_t CAP>
struct Buffer
{
	enum Mode
	{
		LINEAR,
		RING,
	};

	Buffer(Mode mode=RING)
	{
		m_mode = mode;
	}

	void push_back(const T& e)
	{
		if (m_mode == LINEAR && m_size < CAP - 1)
		{
			m_elements[m_writer++] = e;
			m_size++;
		}
		else if (m_mode == RING)
		{
			m_elements[m_writer] = e;
			m_writer = (m_writer + 1) % CAP;
			
			if (m_size >= CAP)
			{
				m_reader = (m_reader + 1) % CAP;      
			}
			else
			{
				m_size++;
			}
		}
	}

	bool peek_front(T& e_out)
	{
		if (m_size == 0) { return false; }

		e_out = m_elements[m_reader];
		return true;
	}


	bool pop_front(T& e_out)
	{
		if (m_mode == RING)
		{
			if (m_size == 0) { return false; }
			e_out = m_elements[m_reader];
			m_reader = (m_reader + 1) % CAP;
			m_size--;  

			return true;  		
		}

		return false;
	}

	size_t size() const { return m_size; }

	T& operator[](unsigned i) { return m_elements[(m_reader + i) % CAP]; }

	T* data() { return m_elements + m_reader; }

	Mode m_mode;
	T m_elements[CAP];
	size_t m_writer = 0, m_reader = 0;
	size_t m_size = 0;
};

static bool read_buffer_ok(Stream& stream, uint8_t* buf, size_t size, int tries=1)
{
	auto bytes_remaining = size;
	for (; bytes_remaining > 0 && tries--;)
	{      
			uint8_t* s_ptr = buf;
			uint8_t* r_ptr = buf;
			while ((r_ptr - s_ptr) < size)
			{
				r_ptr += stream.readBytes(r_ptr, size);
			}
	}

	return true;
}


struct UbloxPacket
{
	struct Header
	{
		uint8_t sync[2];			 // Sync characters. Always 0xB5, 0x62
		uint8_t cls;
		uint8_t id;
		uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
	};

	struct Footer
	{
		uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
		uint8_t checksumB;
	};

	Header header;
	Buffer<uint8_t, 128> payload;
	Footer footer;

	void compute_checksums(uint8_t& checksumA, uint8_t& checksumB) const
	{
		checksumA = 0;
		checksumB = 0;

		checksumA += header.cls;
		checksumB += checksumA;

		checksumA += header.id;
		checksumB += checksumA;

		checksumA += (header.len & 0xFF);
		checksumB += checksumA;

		checksumA += (header.len >> 8);
		checksumB += checksumA;

		for (uint16_t i = 0; i < payload.size(); i++)
		{
			checksumA += payload[i];
			checksumB += checksumA;
		}
	}

	void print() const
	{
			for (unsigned i = 0; i < sizeof(header); i++)
			{ // print header
				Serial.print(((uint8_t*)&header)[i], HEX); Serial.print(" ");
			}

			for (unsigned i = 0; i < header.len; i++)
			{ // print payload
				Serial.print(payload[i], HEX); Serial.print(" ");
			}

			for (unsigned i = 0; i < sizeof(footer); i++)
			{ // print footer
				Serial.print(((uint8_t*)&footer)[i], HEX); Serial.print(" ");
			}
			Serial.println();
	}

	size_t size() const
	{
		return sizeof(header) + sizeof(footer) + payload.size();
	}

	bool is_valid() const
	{
		uint8_t a, b;
		compute_checksums(a, b);

		return a == footer.checksumA && b == footer.checksumB;
	}

	inline uint16_t key() const
	{
		return CAT_FIELDS(header.cls, header.id);
	}

	template<typename T>
	void push_payload(const T& p)
	{
		uint8_t* w_ptr = (uint8_t*)&p;

		Serial.print("pushing payload buffer of: "); Serial.println(sizeof(p));

		for (unsigned i = 0; i < sizeof(p); i++)
		{
			Serial.print(*w_ptr, HEX); Serial.print(" ");
			payload.push_back(*w_ptr++);
		} Serial.println();

		Serial.print("Final size: "); Serial.println(payload.size(), DEC);
	}

	static bool read_from_stream(Stream& stream, UbloxPacket& out)
	{
		uint8_t hdr[16];
		uint8_t payload[32];
		uint8_t footer[8];
		// read(fd, &packet.header, sizeof(packet.header));
		if (!read_buffer_ok(stream, (uint8_t*)&out.header, sizeof(out.header))) { return false; }
		if (!read_buffer_ok(stream, out.payload.data(), out.header.len)) { return false; }
		if (!read_buffer_ok(stream, (uint8_t*)&out.footer, sizeof(out.footer))) { return false; }

		return true;
	}
};

struct ZED_FP9
{
	ZED_FP9(uint8_t address)
		: m_slave_address(address)
	{
		m_port_config.portID = 0x00; // I2C port is always 0
		m_port_config.txReady.bits = {
			0, // tx ready
			0, // polarity 0 High-active, 1 Low-active
			0, // PIO to be used (must not be used for other purpose)
			0, // TX ready threshold in 8 byte increments
		};
		m_port_config.mode = m_slave_address;
		m_port_config.inProtoMask.bits.inUbx = 1;
		m_port_config.inProtoMask.bits.inNmea = 0;
		m_port_config.inProtoMask.bits.inRtcm = 0;
		m_port_config.inProtoMask.bits.inRtcm3 = 0;
		m_port_config.inProtoMask.bits.inSPARTN = 0;
		m_port_config.outProtoMask.bits.outUbx = 1;
		m_port_config.outProtoMask.bits.outNmea = 0;
		m_port_config.outProtoMask.bits.outRtcm3 = 0;
		m_port_config.outProtoMask.bits.outSPARTN = 0;
		m_port_config.flags = 0; // bit 1 set for extended timeout behavior

		m_rate_config.measRate = 1000; // 1 hz
		m_rate_config.navRate = 5; // 5 measurements per navigation solution
		m_rate_config.timeRef = 0; // UTC time
	}


	void service_receive(Stream& stream, size_t bytes)
	{
		if (bytes == 1)
		{ // we are being addressed first
			m_current_register = stream.read();

			Serial.print("Selected reg: "); Serial.println(m_current_register, HEX);
		}
		else if (bytes > 0)
		{
			UbloxPacket rx_packet;

			if (UbloxPacket::read_from_stream(stream, rx_packet))
			{
				rx_packet.print();
				process_packet(stream, rx_packet);
			}
			else
			{

			}
		}

	}

	void service_request(Stream& stream)
	{
		switch(m_current_register)
		{
			case 0xFD:
			{
				uint16_t size = 0;

				UbloxPacket to_send;
				if (m_tx_queue.peek_front(to_send))
				{
					size = to_send.size();
				}
				// for (unsigned i = 0; i < m_tx_queue.size(); i++)
				// {
				// 	size += m_tx_queue[i].size();          
				// }
				Serial.print("Responding with size: "); Serial.println(size);
				stream.write(((uint8_t*)&size)[1]);
				stream.write(((uint8_t*)&size)[0]);
				m_current_register = 0;
			} break;
			default:
			{
					UbloxPacket tx_packet;
					Serial.print("tx_queue size: "); Serial.println(m_tx_queue.size());

					if(m_tx_queue.pop_front(tx_packet))
					{
						transmit_packet(stream, tx_packet);
					}

					if(m_tx_queue.size() == 0)
					{
						Serial.println("tx_queue empty");
					}
			} break;
		}
	}

	void update(unsigned time_ms)
	{
		if (time_ms - m_last_pvt_time > 1000)
		{
			m_last_pvt_time = time_ms;
			if (m_auto_pvt)
			{
				UbloxPacket msg = make_msg(UBX_CLASS_NAV, UBX_NAV_PVT, sizeof(UBX_NAV_PVT_data_t));
				msg.push_payload<>(UBX_NAV_PVT_data_t{});
				m_tx_queue.push_back(msg);
				m_last_pvt_time = time_ms;
			}
		}
	}

private:
	UbloxPacket make_cfg_prt_i2c()
	{
		UbloxPacket cfg_prt_i2c;
		cfg_prt_i2c.header.sync[0] = UBX_SYNCH_1;
		cfg_prt_i2c.header.sync[1] = UBX_SYNCH_2;
		cfg_prt_i2c.header.cls = UBX_CLASS_CFG;
		cfg_prt_i2c.header.id = UBX_CFG_PRT;
		cfg_prt_i2c.header.len = 20;

		cfg_prt_i2c.push_payload<>(m_port_config);
		cfg_prt_i2c.compute_checksums(cfg_prt_i2c.footer.checksumA, cfg_prt_i2c.footer.checksumB);

		return cfg_prt_i2c;
	}

	UbloxPacket make_ack(const UbloxPacket& ack_packet)
	{
		UbloxPacket ack = make_msg(UBX_CLASS_ACK, UBX_ACK_ACK, 2);
		ack.payload.push_back(ack_packet.header.cls);
		ack.payload.push_back(ack_packet.header.id);
		ack.compute_checksums(ack.footer.checksumA, ack.footer.checksumB);

		return ack;
	}

	UbloxPacket make_nack(const UbloxPacket& nack_packet)
	{
		UbloxPacket nack = make_msg(UBX_CLASS_ACK, UBX_ACK_NACK, 2);
		nack.payload.push_back(nack_packet.header.cls);
		nack.payload.push_back(nack_packet.header.id);
		nack.compute_checksums(nack.footer.checksumA, nack.footer.checksumB);

		return nack;
	}

	UbloxPacket make_msg(uint8_t cls, uint8_t id, uint16_t len)
	{
		UbloxPacket msg;
		msg.header.sync[0] = UBX_SYNCH_1;
		msg.header.sync[1] = UBX_SYNCH_2;
		msg.header.cls = cls;
		msg.header.id = id;
		msg.header.len = len;

		return msg;
	}

	static void transmit_packet(Stream& stream, UbloxPacket& packet)
	{
		Serial.println("Transmitting packet:");
		packet.print();

		packet.compute_checksums(packet.footer.checksumA, packet.footer.checksumB);
		stream.write((uint8_t*)&packet.header, sizeof(packet.header));
		for (unsigned i = 0; i < packet.header.len; i++)
		{
			stream.write(packet.payload[i]);
		}
		// stream.write(packet.payload.data(), packet.header.len);
		stream.write((uint8_t*)&packet.footer, sizeof(packet.footer));
	}

	void process_cfg_nav(const UbloxPacket& packet)
	{
		auto msg_id = packet.payload[1];

		switch (msg_id)
		{
			case UBX_NAV_PVT:
				m_rate_config.navRate = packet.payload[2];
				m_auto_pvt = true;
				break;
			default:
				break;
		}
	}

	void process_cfg(Stream& stream, const UbloxPacket& packet)
	{
		switch(packet.header.id)
		{
			case UBX_CFG_PRT:
			{
				Serial.println("::: config port");

				if (packet.header.len == 1)
				{ // this is a polling packet, so respond with the current config
					Serial.println("Sending master port config");
					m_tx_queue.push_back(make_ack(packet));
					m_tx_queue.push_back(make_cfg_prt_i2c());
				}
				else if(packet.header.len == sizeof(UBX_CFG_PRT_data_t))
				{
					Serial.println("setting new port config");
					// this is a config packet, so store the new config
					UBX_CFG_PRT_data_t* cfg = (UBX_CFG_PRT_data_t*)packet.payload.data();
					m_port_config = *cfg;
					m_tx_queue.push_back(make_ack(packet));
				}
				else
				{ // this is an invalid packet, so send a NACK
					m_tx_queue.push_back(make_nack(packet));
				}

			} break;

			case UBX_CFG_MSG:
			{
				Serial.println("::: config message rate");

				if(packet.header.len == 3)
				{
					auto msg_class = packet.payload[0];
					auto msg_id = packet.payload[1];

					switch(msg_class)
					{
						case UBX_CLASS_NAV:
							process_cfg_nav(packet);
							break;
						default:
							m_message_rate = packet.payload[2];
							break;
					}

					Serial.print("message rate set to: "); Serial.println(m_message_rate);

					m_tx_queue.push_back(make_ack(packet));
				}
				else
				{
					m_tx_queue.push_back(make_nack(packet));
				}
			} break;

			case UBX_CFG_RATE:
			{
				Serial.println("::: config navigation rate");

				if(packet.header.len == 0)
				{ // send back the current navigation rate
					Serial.println("Sending current navigation rate");
					UbloxPacket msg = make_msg(UBX_CLASS_CFG, UBX_CFG_RATE, sizeof(m_rate_config));
					msg.push_payload<>(m_rate_config);
					m_tx_queue.push_back(msg);
					m_tx_queue.push_back(make_ack(packet));
				}
				else if (packet.header.len == sizeof(UBX_CFG_RATE_data_t))
				{
					m_tx_queue.push_back(make_ack(packet));

					m_rate_config = *(UBX_CFG_RATE_data_t*)packet.payload.data();

					Serial.println("Set navigation rate rates to:");
					Serial.print("measRate: "); Serial.println(m_rate_config.measRate);
					Serial.print("navRate: "); Serial.println(m_rate_config.navRate);
					Serial.print("timeRef: "); Serial.println(m_rate_config.timeRef);
				}
				else
				{
					m_tx_queue.push_back(make_nack(packet));
				}
			} break;
		}

	}

	void process_packet(Stream& stream, const UbloxPacket& packet)
	{
		switch(packet.header.cls)
		{
			case UBX_CLASS_CFG:
				process_cfg(stream, packet);
				break;
		}
	}

	bool m_auto_pvt = false; // automatically send PVT messages
	UBX_CFG_PRT_data_t m_port_config = {};
	UBX_CFG_RATE_data_t m_rate_config = {};
	uint8_t m_message_rate;

	Buffer<UbloxPacket, 3> m_tx_queue;
	uint8_t m_current_register;
	uint8_t m_slave_address;
	unsigned long m_last_pvt_time = 0;
};

} // namespace emulation
