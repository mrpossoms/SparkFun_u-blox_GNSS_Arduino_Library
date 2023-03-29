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

  T* data() { return m_elements; }

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
        Serial.print(payload.data()[i], HEX); Serial.print(" ");
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

      // uint8_t rx_buf[128];
      
      // uint8_t* s_ptr = rx_buf;
      // uint8_t* r_ptr = rx_buf;
      // while ((r_ptr - s_ptr) < bytes)
      // {
      //   r_ptr += Wire.readBytes(r_ptr, bytes);
      // }

      // for (unsigned i = 0; i < bytes; i++)
      // {
      //   Serial.print(rx_buf[i], HEX); Serial.print(" ");
      // } Serial.println();
    }

	}

	void service_request(Stream& stream)
	{
		switch(m_current_register)
		{
			case 0xFD:
			{
				uint16_t size = 0;
				for (unsigned i = 0; i < m_tx_queue.size(); i++)
				{
					size += m_tx_queue[i].size();          
				}
        Serial.print("Responding with size: "); Serial.println(size);
        stream.write(((uint8_t*)&size)[1]);
        stream.write(((uint8_t*)&size)[0]);
        m_current_register = 0;
			} break;
			default:
			{
					UbloxPacket tx_packet;
					while(m_tx_queue.pop_front(tx_packet));
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

private:
	static UbloxPacket make_ack(const UbloxPacket& ack_packet)
	{
		UbloxPacket ack;
		ack.header.sync[0] = UBX_SYNCH_1;
		ack.header.sync[1] = UBX_SYNCH_2;
		ack.header.cls = UBX_CLASS_ACK;
		ack.header.id = UBX_ACK_ACK;
		ack.header.len = 2;
		ack.payload.push_back(ack_packet.header.cls);
		ack.payload.push_back(ack_packet.header.id);
		ack.compute_checksums(ack.footer.checksumA, ack.footer.checksumB);

		return ack;
	}

	static UbloxPacket make_nack(const UbloxPacket& nack_packet)
	{
		UbloxPacket nack;
		nack.header.sync[0] = UBX_SYNCH_1;
		nack.header.sync[1] = UBX_SYNCH_2;
		nack.header.cls = UBX_CLASS_ACK;
		nack.header.id = UBX_ACK_NACK;
		nack.header.len = 2;
		nack.payload.push_back(nack_packet.header.cls);
		nack.payload.push_back(nack_packet.header.id);
		nack.compute_checksums(nack.footer.checksumA, nack.footer.checksumB);

		return nack;
	}

	static void transmit_packet(Stream& stream, const UbloxPacket& packet)
	{
		UbloxPacket::Header hdr = packet.header;
		Serial.println("Transmitting packet:");
		packet.print();
		//hdr.len = ((hdr.len & 0xFF) << 8) || (hdr.len >> 8);
		stream.write((uint8_t*)&hdr, sizeof(hdr));
		stream.write(packet.payload.data(), packet.header.len);
		stream.write((uint8_t*)&packet.footer, sizeof(packet.footer));
	}


	void process_cfg(Stream& stream, const UbloxPacket& packet)
	{
		bool ack = false;

		switch(packet.header.id)
		{
			case 0x00:
				Serial.println("Enqueuing ack");
				ack = true;
				break;
		}

		if (ack)
		{
			m_tx_queue.push_back(make_ack(packet));
			// transmit_packet(stream, make_ack());
		}
		else
		{
			m_tx_queue.push_back(make_nack(packet));
			// transmit_packet(stream, make_nack());
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

	//std::queue<UbloxPacket> m_packet_queue;
  Buffer<UbloxPacket, 3> m_tx_queue;
  uint8_t m_current_register;
};

} // namespace emulation
