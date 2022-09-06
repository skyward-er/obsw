/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <mxgui/display.h>
#include <radio/SX1278/SX1278.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/ScreenManager.h>
#include <utils/gui/VerticalLayout.h>

std::string format_link_speed(size_t value)
{
    if (value > 1000000)
        return fmt::format("{:.2f} mb/s", static_cast<float>(value) / 1000000);
    else if (value > 1000)
        return fmt::format("{:.2f} kb/s", static_cast<float>(value) / 1000);
    else
        return fmt::format("{} b/s", value);
}

class StatsScreen
{
public:
    struct Data
    {
        size_t tx_bitrate;
        size_t rx_bitrate;
        int sent_count;
        int recv_count;
        float rssi;
    };

    StatsScreen()
    {
        status.setAlignment(Boardcore::HorizAlignment::CENTER,
                            Boardcore::VertAlignment::CENTER);
        status.setBackgroundColor(mxgui::red);
        status.setTextColor(mxgui::black);
        status.setFont(mxgui::miscFixedBold);

        body.setCell(&lbl_tx_bitrate, 0, 0);
        body.setCell(&lbl_sent_count, 1, 0);
        body.setCell(&lbl_rx_bitrate, 2, 0);
        body.setCell(&lbl_recv_count, 3, 0);
        body.setCell(&lbl_rssi, 4, 0);

        body.setCell(&tx_bitrate, 0, 1);
        body.setCell(&sent_count, 1, 1);
        body.setCell(&rx_bitrate, 2, 1);
        body.setCell(&recv_count, 3, 1);
        body.setCell(&rssi, 4, 1);

        root.addView(&status, 0.1);
        root.addView(&body, 1.0);
    }

    void updateError(Boardcore::SX1278::Error value)
    {
        status.setBackgroundColor(mxgui::red);
        switch (value)
        {
            case Boardcore::SX1278::Error::BAD_VALUE:
                status.setText("BAD VALUE");
                break;
            case Boardcore::SX1278::Error::BAD_VERSION:
                status.setText("BAD VERSION");
                break;
            default:
                break;
        }
    }

    void updateReady()
    {
        status.setBackgroundColor(mxgui::green);
        status.setText("READY");
    }

    void updateStats(const Data &data) {
        tx_bitrate.setText(format_link_speed(data.tx_bitrate));
        sent_count.setText(fmt::format("{}", data.sent_count));

        rx_bitrate.setText(format_link_speed(data.rx_bitrate));
        recv_count.setText(fmt::format("{}", data.recv_count));

        rssi.setText(fmt::format("{} dBm", data.rssi));
    }

    Boardcore::VerticalLayout root{10};
    Boardcore::TextView status{"LOADING"};

    Boardcore::GridLayout body{5, 2};
    Boardcore::TextView lbl_tx_bitrate{"Tx bitrate:"};
    Boardcore::TextView lbl_sent_count{"Packet sent:"};
    Boardcore::TextView lbl_rx_bitrate{"Rx bitrate:"};
    Boardcore::TextView lbl_recv_count{"Packet received:"};
    Boardcore::TextView lbl_rssi{"RSSI:"};

    Boardcore::TextView tx_bitrate{"0 b/s"};
    Boardcore::TextView sent_count{"0"};
    Boardcore::TextView rx_bitrate{"0 b/s"};
    Boardcore::TextView recv_count{"0"};
    Boardcore::TextView rssi{"0 dBm"};
};

class GUI
{
public:
    GUI() : screen_manager(mxgui::DisplayManager::instance(), 8)
    {
        screen_manager.addScreen(0, &stats_screen.root);
        screen_manager.start();
    }

    ~GUI() { screen_manager.stop(); }

    Boardcore::ScreenManager screen_manager;
    StatsScreen stats_screen;
};