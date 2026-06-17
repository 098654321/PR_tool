#include "xinzhai_register.hh"
#include <format>


namespace PR_tool::parse
{
    XinzhaiRegister::~XinzhaiRegister() noexcept {
        this->_pinterposer = nullptr;
    }

    auto XinzhaiRegister::fetch_controlbits(RegisterValue& rv) -> void
    {
        fetch_padctrl_right(rv.xinzhai.padctrl_right);
        fetch_padctrl_left(rv.xinzhai.padctrl_left);
        fetch_padctrl_up(rv.xinzhai.padctrl_up);
        fetch_padctrl_down(rv.xinzhai.padctrl_down);

        fetch_SiPpadctrl_right(rv.xinzhai.SiPpadctrl_right);
        fetch_SiPpadctrl_left(rv.xinzhai.SiPpadctrl_left);
        fetch_SiPpadctrl_up(rv.xinzhai.SiPpadctrl_up);
        fetch_SiPpadctrl_down(rv.xinzhai.SiPpadctrl_down);
    }

    auto XinzhaiRegister::fetch_padctrl_right(std::Bits<128>& bits) -> void {
        // internal [2,11].up_sel -> PR cob_6_11 right
        this->fetch_padctrl_template(bits, 6, 11, hardware::COBDirection::Right);
    }

    auto XinzhaiRegister::fetch_padctrl_left(std::Bits<128>& bits) -> void {
        // internal [8,2].right_sel -> PR cob_0_2 down
        this->fetch_padctrl_template(bits, 0, 2, hardware::COBDirection::Down);
    }

    auto XinzhaiRegister::fetch_padctrl_up(std::Bits<128>& bits) -> void {
        // internal [0,1].left_sel -> PR cob_8_1 up
        this->fetch_padctrl_template(bits, 8, 1, hardware::COBDirection::Up);
    }

    auto XinzhaiRegister::fetch_padctrl_down(std::Bits<128>& bits) -> void {
        // internal [8,10].right_sel -> PR cob_0_10 down
        this->fetch_padctrl_template(bits, 0, 10, hardware::COBDirection::Down);
    }

    auto XinzhaiRegister::fetch_SiPpadctrl_right(std::Bits<128>& bits) -> void {
        // internal [0,8].left_sel -> PR cob_8_8 up
        this->fetch_SiPpadctrl_template(bits, 8, 8, hardware::COBDirection::Up);
    }

    auto XinzhaiRegister::fetch_SiPpadctrl_left(std::Bits<128>& bits) -> void {
        // internal [4,0].down_sel -> PR cob_4_0 left
        this->fetch_SiPpadctrl_template(bits, 4, 0, hardware::COBDirection::Left);
    }

    auto XinzhaiRegister::fetch_SiPpadctrl_up(std::Bits<128>& bits) -> void {
        // internal [0,5].left_sel base; golden also overrides [0,4] at 0/8/16/24 but
        // only port indices in cob_ext_port_index affect IE field output.
        this->fetch_SiPpadctrl_template(bits, 8, 5, hardware::COBDirection::Up);
    }

    auto XinzhaiRegister::fetch_SiPpadctrl_down(std::Bits<128>& bits) -> void {
        // internal [8,6].right_sel -> PR cob_0_6 down
        this->fetch_SiPpadctrl_template(bits, 0, 6, hardware::COBDirection::Down);
    }

    auto XinzhaiRegister::fetch_padctrl_template(std::Bits<128>& bits, std::i64 row, std::i64 col, hardware::COBDirection dir) -> void
    {
        auto pcob = _pinterposer->get_cob(row, col);
        if (!pcob.has_value())
        {
            throw std::logic_error(std::format("cob at ({}, {}) does not exist", row, col));
        }
        auto cob = pcob.value();

        // group 1
        for (std::size_t index = 127; index >= 96; --index) {
            bits[index] = 1;
        }

        // group 2
        for (std::size_t index = 95; index >= 64; --index) {
            bits[index] = 0;
        }

        // group 3
        for (std::size_t index = 63; index >= 48; --index) {
            std::size_t port_index = this->_cob_pad_ext_port_index[15 - (index - 48)];
            auto value = cob->get_sel_resgiter_value(dir, port_index);
            if (value == hardware::COBSignalDirection::TrackToCOB)
            {
                bits[index] = 1;
            }
            else
            {
                bits[index] = 0;
            }
        }

        // group 4
        for (std::size_t index = 47; index >= 32; --index) {
            bits[index] = 0;
        }

        // group 5
        for (int index = 31; index >= 0; --index) {
            bits[index] = 0;
        }
    }

    auto XinzhaiRegister::fetch_SiPpadctrl_template(std::Bits<128>& bits, std::i64 row, std::i64 col, hardware::COBDirection dir) -> void
    {
        auto pcob = _pinterposer->get_cob(row, col);
        if (!pcob.has_value())
        {
            throw std::logic_error(std::format("cob at ({}, {}) does not exist", row, col));
        }
        auto cob = pcob.value();

        // group 1
        for (std::size_t index = 127; index >= 96; --index) {
            bits[index] = 1;
        }

        // group 2
        for (std::size_t index = 95; index >= 64; --index) {
            bits[index] = 0;
        }

        // group 3
        for (std::size_t index = 63; index >= 48; --index) {
            std::size_t port_index = this->_cob_Sip_ext_port_index[15 - (index - 48)];
            auto value = cob->get_sel_resgiter_value(dir, port_index);
            if (value == hardware::COBSignalDirection::TrackToCOB)
            {
                bits[index] = 1;
            }
            else
            {
                bits[index] = 0;
            }
        }

        // group 4
        for (std::size_t index = 47; index >= 32; --index) {
            bits[index] = 0;
        }

        // group 5
        for (int index = 31; index >= 0; --index) {
            bits[index] = 0;
        }
    }
    
}

