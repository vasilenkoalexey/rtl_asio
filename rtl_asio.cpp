// MIT License

// Copyright (c) 2022 Vasilenko Alexey

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <array>
#include <charconv>
#include <iostream>
#include <thread>
#include <optional>

#include <signal.h>

#include <asio.hpp>

extern "C" {
#include "rtl-sdr.h"
}
#define DEFAULT_BUF_LENGTH	(16 * 32 * 512)

static int set_gain_by_index(rtlsdr_dev_t* dev, const unsigned int index) {
    if (int count = rtlsdr_get_tuner_gains(dev, nullptr); count > 0 && static_cast<unsigned int>(count) > index) {
        const auto gains(std::make_unique<int[]>(count));
        count = rtlsdr_get_tuner_gains(dev, gains.get());
        return rtlsdr_set_tuner_gain(dev, gains[index]);
    }
    return 0;
}

static std::optional<uint32_t> verbose_device_search(const std::string_view s) {
    char vendor[256], product[256], serial[256];
    const uint32_t device_count = rtlsdr_get_device_count();
    if (!device_count) {
        std::cout << "No supported devices found." << std::endl;
        return std::nullopt;
    }
    std::cout << "Found " << device_count << " device(s):" << std::endl;
    uint32_t device;
    for (device = 0; device < device_count; device++) {
        rtlsdr_get_device_usb_strings(device, vendor, product, serial);
        std::cout << "  " << device << ":  " << vendor << ", " << product << ", SN: " << serial << std::endl;
    }
    std::cout << std::endl;
    /* does string look like raw id number */
    auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), device);
    if (ec == std::errc() && device < device_count) {
        std::cout << "Using device " << device << ": " << rtlsdr_get_device_name(device) << std::endl;
        return device;
    }
    /* does string exact match a serial */
    for (device = 0; device < device_count; device++) {
        rtlsdr_get_device_usb_strings(device, vendor, product, serial);
        if(std::string(serial) == s) {
            std::cout << "Using device " << device << ": " << rtlsdr_get_device_name(device) << std::endl;
            return device;
        }
    }
    /* does string prefix or suffix match a serial */    
    for (device = 0; device < device_count; device++) {
        rtlsdr_get_device_usb_strings(device, vendor, product, serial);
        if(std::string(serial).find(s) != std::string::npos) {
            std::cout << "Using device " << device << ": " << rtlsdr_get_device_name(device) << std::endl;
            return device;
        }
    }    
    std::cout << "No matching devices found." << std::endl;
    return std::nullopt;
}

/* standard suffixes */
static double atofs(const std::string_view s) {
    double suff = 1.0;
    double res = 0.0;
    switch (s.back()) {
        case 'g':
        case 'G':
            suff *= 1e3;
            /* fall-through */
        case 'm':
        case 'M':
            suff *= 1e3;
            /* fall-through */
        case 'k':
        case 'K':
            suff *= 1e3;
    }
    std::from_chars(s.data(), s.data() + s.size(), res);
    return suff * res;
}

template <std::size_t SIZE>
int get_string_descriptor(int pos, const std::array<uint8_t, SIZE>& data, std::string& str) {
    const int len = data.at(pos);
    if (data.at(pos + 1) != 0x03) {
        std::cout << "Error: invalid string descriptor!" << std::endl;
    }
    int i;
    for (i = 2; i < len; i += 2) {
        str.push_back(data.at(pos + i));
    }
    return pos + i;
}

void rtlsdr_callback(unsigned char* buf, uint32_t len, void* ctx) {
    (*static_cast<std::function<void(const unsigned char*, const uint32_t)>*>(ctx))(buf, len);
}

int main(int argc, char* argv[]) {
    int enable_biastee = 0;
    uint32_t frequency = 100000000, samp_rate = 2048000;
    std::optional<uint32_t> dev_index = 0;
    int dev_given = 0;
    int gain = 0;
    int ppm_error = 0;
    uint32_t buf_num = 0;
    std::string addr = "0.0.0.0";
    std::string port = "1234";

    const std::vector<std::string_view> args(argv + 1, argv + argc);
    for (auto it = args.begin(), end = args.end(); it != end; ++it) {
        const auto opt_name = *it;
        if (opt_name.back() == 'T') {
            enable_biastee = 1;
            continue;
        }
        if (it + 1 != end) {
            ++it;
            const auto opt_value = *it;
            switch (opt_name.back()) {
                case 'd':
                    dev_index = verbose_device_search(opt_value.data());
                    dev_given = 1;
                    continue;
                case 'f':
                    frequency = static_cast<uint32_t>(atofs(opt_value));
                    continue;
                case 'g': {
                    float tmp;
                    if (std::from_chars(opt_value.data(), opt_value.data() + opt_value.size(), tmp).ec == std::errc()) {
                        gain = static_cast<int>(tmp * 10); /* tenths of a dB */
                    }
                }
                    continue;
                case 's':
                    samp_rate = static_cast<uint32_t>(atofs(opt_value));
                    continue;
                case 'a':
                    addr = opt_value;
                    continue;
                case 'p':
                    port = opt_value;
                    continue;
                case 'b':
                    std::from_chars(opt_value.data(), opt_value.data() + opt_value.size(), buf_num);
                    continue;
                case 'P':
                    std::from_chars(opt_value.data(), opt_value.data() + opt_value.size(), ppm_error);
                    continue;
            }
        }
        std::cout << "rtl_asio, an I/Q spectrum server for RTL2832 based DVB-T receivers" << std::endl
                  << std::endl;
        std::cout << "Usage:\t[-a listen address]" << std::endl;
        std::cout << "\t[-p listen port (default: 1234)]" << std::endl;
        std::cout << "\t[-f frequency to tune to [Hz]]" << std::endl;
        std::cout << "\t[-g gain (default: 0 for auto)]" << std::endl;
        std::cout << "\t[-s samplerate in Hz (default: 2048000 Hz)]" << std::endl;
        std::cout << "\t[-b number of buffers (default: 15, set by library)]" << std::endl;
        std::cout << "\t[-d device index (default: 0)]" << std::endl;
        std::cout << "\t[-P ppm_error (default: 0)]" << std::endl;
        std::cout << "\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]" << std::endl;
        return 0;
    }

    if (!dev_given) {
        dev_index = verbose_device_search("0");
    }

    if (!dev_index) {
        return 0;
    }

    rtlsdr_dev_t* dev = nullptr;

    asio::io_context io_context;
    asio::ip::tcp::socket rtl_socket{io_context};
    asio::signal_set signals(io_context, SIGINT, SIGTERM);
#if defined(SIGQUIT)
    signals.add(SIGQUIT);
#endif
    std::array<std::atomic_uint32_t, 14> cmd;

    uint8_t cmd_value = 0;
    uint32_t param_value = 0;
    std::array<asio::mutable_buffer, 2> cmd_buf{asio::buffer(&cmd_value, sizeof(cmd_value)), asio::buffer(&param_value, sizeof(param_value))};

    std::function<void()> do_read;
    std::function<void()> do_accept;

    std::function<void(const unsigned char*, const uint32_t)> do_write = [&](const unsigned char* buf, const uint32_t len) {
        static bool write_ready = true;
        if (write_ready) {
            write_ready = false;
            static char* iq_buf = new char[DEFAULT_BUF_LENGTH];            
            std::memcpy(iq_buf, buf, len);
            asio::async_write(rtl_socket, asio::buffer(iq_buf, DEFAULT_BUF_LENGTH), [](auto, auto) { write_ready = true; });
        }
    };

    std::atomic<uint16_t> flag_rtl{0};

    uint32_t tuner_type = 0;
    uint32_t tuner_gains = 0;
    const std::array<asio::const_buffer, 3> rtl_buf{asio::buffer("RTL0", 4), asio::buffer(&tuner_type, 4), asio::buffer(&tuner_gains, 4)};

    do_accept = [&]() {
        const asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string(addr), std::stoi(port));
        const auto rtl_acceptor(std::make_shared<asio::ip::tcp::acceptor>(io_context, endpoint));
        signals.cancel();
        signals.async_wait([rtl_acceptor](std::error_code ec, auto) {
            if (!ec) {
                std::cout << "Signal caught, exiting!" << std::endl;
                rtl_acceptor->close();
            }
        });
        std::cout << "listening..." << std::endl;
        rtl_acceptor->async_accept([&](std::error_code ec, asio::ip::tcp::socket socket) {
            if (ec) {
                return;
            }
            signals.cancel();
            std::cout << "client accepted! " << socket.remote_endpoint().address() << " " << socket.remote_endpoint().port() << std::endl;
            rtl_socket = std::move(socket);
            signals.async_wait([&](std::error_code ec, auto) {
                if (!ec) {
                    std::cout << "Signal caught, exiting!" << std::endl;
                    rtl_socket.close();
                }
            });
            rtlsdr_open(&dev, dev_index.value());
            if (nullptr == dev) {
                std::cout << "Failed to open rtlsdr device #" << dev_index.value() << "." << std::endl;
                do_accept();
                return;
            }

            std::array<uint8_t, 256> eeprom_buf;
            if (const int r = rtlsdr_read_eeprom(dev, eeprom_buf.data(), 0, 256); r < 0) {
                if (r == -3) {
                    std::cout << "No EEPROM has been found." << std::endl;
                } else {
                    std::cout << "Failed to read EEPROM, err " << r << "." << std::endl;
                }
            } else {
                if ((eeprom_buf.at(0) != 0x28) || (eeprom_buf.at(1) != 0x32)) {
                    std::cout << "Error: invalid RTL2832 EEPROM header!" << std::endl;
                } else {
                    std::string manufacturer;
                    int pos = get_string_descriptor(0x09, eeprom_buf, manufacturer);
                    std::string product;    
                    pos = get_string_descriptor(pos, eeprom_buf, product);
                    std::string serial;    
                    get_string_descriptor(pos, eeprom_buf, serial);

                    std::cout << "Vendor ID:\t\t\t0x" << std::hex << (eeprom_buf.at(2) | (eeprom_buf.at(3) << 8)) << std::endl;
                    std::cout << "Product ID:\t\t\t0x" << std::hex << (eeprom_buf.at(4) | (eeprom_buf.at(5) << 8))  << std::endl;    
                    std::cout << "Manufacturer:\t\t\t" << manufacturer << std::endl;    
                    std::cout << "Product:\t\t\t" << product << std::endl;    
                    std::cout << "Serial number:\t\t\t" << serial << std::endl;    
                    std::cout << "Serial number enabled:\t\t" << ((eeprom_buf.at(6) == 0xa5) ? "yes": "no") << std::endl;
                    std::cout << "Bias-T Always ON:\t\t" << ((eeprom_buf.at(7) & 0x02) ? "no": "yes") << std::endl;
                    std::cout << "Direct Sampling Always ON:\t" << ((eeprom_buf.at(7) & 0x01) ? "yes": "no") << std::endl;
                    std::cout << std::dec;
                }
            }

            if (ppm_error != 0) {
                /* Set the tuner error */
                int r = rtlsdr_set_freq_correction(dev, ppm_error);
                if (r < 0) {
                    std::cout << "WARNING: Failed to set freq correction." << std::endl;
                }
            }

            /* Set the sample rate */
            if (const int r = rtlsdr_set_sample_rate(dev, samp_rate); r < 0) {
                std::cout << "WARNING: Failed to set sample rate." << std::endl;
            }

            /* Set the frequency */
            if (const int r = rtlsdr_set_center_freq(dev, frequency); r < 0) {
                std::cout << "WARNING: Failed to set center freq." << std::endl;
            } else {
                std::cout << "Tuned to " << frequency << " Hz." << std::endl;
            }

            if (0 == gain) {
                /* Enable automatic gain */
                if (const int r = rtlsdr_set_tuner_gain_mode(dev, 0); r < 0) {
                    std::cout << "WARNING: Failed to enable automatic gain.\n"
                              << std::endl;
                }
            } else {
                /* Enable manual gain */
                if (const int r = rtlsdr_set_tuner_gain_mode(dev, 1); r < 0) {
                    std::cout << "WARNING: Failed to enable manual gain." << std::endl;
                }
                /* Set the tuner gain */
                if (const int r = rtlsdr_set_tuner_gain(dev, gain); r < 0) {
                    std::cout << "WARNING: Failed to set tuner gain." << std::endl;
                } else {
                    std::cout << "Tuner gain set to " << gain / 10.0 << "dB." << std::endl;
                }
            }

            rtlsdr_set_bias_tee(dev, enable_biastee);
            if (enable_biastee) {
                std::cout << "Activated bias-T on GPIO PIN 0" << std::endl;
            }

            /* Reset endpoint before we start reading from it (mandatory) */
            if (const int r = rtlsdr_reset_buffer(dev); r < 0) {
                std::cout << "WARNING: Failed to reset buffers." << std::endl;
            }

            tuner_type = asio::detail::socket_ops::host_to_network_long(rtlsdr_get_tuner_type(dev));
            tuner_gains = asio::detail::socket_ops::host_to_network_long(rtlsdr_get_tuner_gains(dev, nullptr));

            asio::async_write(rtl_socket, rtl_buf, [&](std::error_code ec, auto) {
                if (!ec) {
                    std::atomic<bool> flag_th(false);
                    static std::thread th;
                    th = std::thread([&]() {
                        std::thread th = std::thread([&]() {
                            flag_th.store(true);
                            flag_th.notify_one();
                            rtlsdr_read_async(dev, rtlsdr_callback, &do_write, buf_num, DEFAULT_BUF_LENGTH);
                            flag_rtl.store(1 << 14);
                            flag_rtl.notify_one();
                        });
                        for (;;) {
                            flag_rtl.wait(0);
                            const auto old = flag_rtl.exchange(0);
                            if (old & (1 << 0)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(0).load());
                                std::cout << "set freq " << tmp << std::endl;
                                rtlsdr_set_center_freq(dev, tmp);
                            }
                            if (old & (1 << 1)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(1).load());
                                std::cout << "set sample rate " << tmp << std::endl;
                                rtlsdr_set_sample_rate(dev, tmp);
                            }
                            if (old & (1 << 2)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(2).load());
                                std::cout << "set gain mode " << tmp << std::endl;
                                rtlsdr_set_tuner_gain_mode(dev, tmp);
                            }
                            if (old & (1 << 3)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(3).load());
                                std::cout << "set gain " << tmp << std::endl;
                                rtlsdr_set_tuner_gain(dev, tmp);
                            }
                            if (old & (1 << 4)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(4).load());
                                std::cout << "set freq correction " << tmp << std::endl;
                                rtlsdr_set_freq_correction(dev, tmp);
                            }
                            if (old & (1 << 5)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(5).load());
                                std::cout << "set if stage " << (tmp >> 16) << "gain " << static_cast<short>(tmp & 0xffff) << std::endl;
                                rtlsdr_set_tuner_if_gain(dev, tmp >> 16, static_cast<short>(tmp & 0xffff));
                            }
                            if (old & (1 << 6)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(6).load());
                                std::cout << "set test mode " << tmp << std::endl;
                                rtlsdr_set_testmode(dev, tmp);
                            }
                            if (old & (1 << 7)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(7).load());
                                std::cout << "set agc mode " << tmp << std::endl;
                                rtlsdr_set_agc_mode(dev, tmp);
                            }
                            if (old & (1 << 8)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(8).load());
                                std::cout << "set direct sampling " << tmp << std::endl;
                                rtlsdr_set_direct_sampling(dev, tmp);
                            }
                            if (old & (1 << 9)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(9).load());
                                std::cout << "set offset tuning " << tmp << std::endl;
                                rtlsdr_set_offset_tuning(dev, tmp);
                            }
                            if (old & (1 << 10)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(10).load());
                                std::cout << "set rtl xtal " << tmp << std::endl;
                                rtlsdr_set_xtal_freq(dev, tmp, 0);
                            }
                            if (old & (1 << 11)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(11).load());
                                std::cout << "set tuner xtal " << tmp << std::endl;
                                rtlsdr_set_xtal_freq(dev, 0, tmp);
                            }
                            if (old & (1 << 12)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(12).load());
                                std::cout << "set tuner gain by index " << tmp << std::endl;
                                set_gain_by_index(dev, tmp);
                            }
                            if (old & (1 << 13)) {
                                const uint32_t tmp = asio::detail::socket_ops::network_to_host_long(cmd.at(13).load());
                                std::cout << "set bias tee " << tmp << std::endl;
                                rtlsdr_set_bias_tee(dev, static_cast<int>(tmp));
                            }
                            if (old & (1 << 14)) {
                                th.join();
                                std::cout << "Close rtlsdr device #" << dev_index.value() << "." << std::endl;
                                rtlsdr_close(dev);
                                return;
                            }
                        }
                    });
                    flag_th.wait(false);
                    do_read = [&]() {
                        asio::async_read(rtl_socket, cmd_buf, [&](std::error_code ec, auto) {
                            if (!ec) {
                                if (const unsigned int index = cmd_value - 1; index < 14) {
                                    flag_rtl.fetch_or(1 << index);
                                    cmd.at(index).store(param_value);
                                    flag_rtl.notify_one();
                                }
                                do_read();
                            } else {
                                rtlsdr_cancel_async(dev);
                                th.join();
                                std::cout << "all threads dead." << std::endl;
                                if (asio::error::operation_aborted != ec.value()) {
                                    do_accept();
                                }
                            }
                        });
                    };                    
                    do_read();
                } else {
                    std::cout << "failed to send dongle information" << std::endl;
                    rtlsdr_close(dev);
                    do_accept();
                }
            });
        });
    };
    do_accept();
    io_context.run();
    std::cout << "bye!" << std::endl;
    return 0;
}