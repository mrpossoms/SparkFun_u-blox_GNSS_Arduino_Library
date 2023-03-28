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
    }
  }

  bool pop_front(T& e_out)
  {
    if (m_size == 0) { return false; }

    e_out = m_elements[m_reader++];
    m_size--;  

    return true;
  }

  size_t size() const { return m_size; }

  T& operator[](unsigned i) { return m_elements[i]; }

  T* data() { return m_elements; }

  Mode m_mode;
  T m_elements[CAP];
  size_t m_writer = 0, m_reader = 0;
  size_t m_size = 0;
};

static bool read_buffer_ok(Stream& stream, uint8_t* buf, size_t size, int tries=3)
{
	auto bytes_remaining = size;

	for (tries = 3; bytes_remaining > 0 && tries--;)
	{
		auto read_res = stream.readBytes(buf + size - bytes_remaining, bytes_remaining);
		if (read_res > 0)
		{
			bytes_remaining -= read_res;
		} 
		else
		{
			// Read failed, throw ex
			return false;
		}
	}

	return tries >= 0;
}



struct UbloxPacket
{
	struct Header
	{
		uint8_t cls;
		uint8_t id;
		uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
		uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
		uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
	} __attribute__((packed));

	struct Footer
	{
		uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
		uint8_t checksumB;
		sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
		sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
	} __attribute__((packed));

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

		checksumA += (payload.size() & 0xFF);
		checksumB += checksumA;

		checksumA += (payload.size() >> 8);
		checksumB += checksumA;

		for (uint16_t i = 0; i < payload.size(); i++)
		{
			checksumA += payload[i];
			checksumB += checksumA;
		}
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
		// read(fd, &packet.header, sizeof(packet.header));
		if (!read_buffer_ok(stream, (uint8_t*)&out.header, sizeof(out.header))) { return false; }
		// out.payload.reserve(out.header.len);
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
    else
    {
      uint8_t rx_buf[128];
      
      uint8_t* s_ptr = rx_buf;
      uint8_t* r_ptr = rx_buf;
      while ((r_ptr - s_ptr) < bytes)
      {
        r_ptr += Wire.readBytes(r_ptr, bytes);
      }

      for (unsigned i = 0; i < bytes; i++)
      {
        Serial.print(rx_buf[i], HEX); Serial.print(" ");
      } Serial.println();
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
        stream.write((uint8_t*)&size, sizeof(size));
        Serial.print("Responding with size: "); Serial.println(size);
			} break;
		}
	}

private:
	void process_packet(Stream& stream, const UbloxPacket& packet)
	{
		switch(packet.key())
		{
			case CAT_FIELDS(UBX_CLASS_CFG, UBX_CFG_PRT):
				// process port config
				break;

		}
	}

	//std::queue<UbloxPacket> m_packet_queue;
  Buffer<UbloxPacket, 3> m_tx_queue;
  uint8_t m_current_register;
};

} // namespace emulation