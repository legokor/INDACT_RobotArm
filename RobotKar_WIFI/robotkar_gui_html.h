/**
 *******************************************************************************
 * @file robotkar_gui_html.h
 *
 * @date Jan 20, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief
 * This file contains the HTML pages used by the webserver.
 *******************************************************************************
 */

static const char control_page_before_status[] PROGMEM =
"<!DOCTYPE html>\
<html>\
<head>\
    <title>INDACT Robotkar</title>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <style>\
        .status {\
            font-size: 24px;\
            font-family: 'Courier New', Courier, monospace;\
            background-color: lightgray;\
            padding: 10px;\
            margin: 10px;\
        }\
        .controls {\
            border: 2px solid black;\
            border-collapse: collapse;\
            width: 100%;\
        }\
        .controls th,\
        .controls td {\
            border-bottom: 1px solid #ddd;\
            width: 50%;\
        }\
        .controls th {\
            font-size: 24px;\
        }\
        .controls button {\
            border: 1px solid black;\
            color: black;\
            width: 80px;\
            margin: 4px;\
            padding: 10px;\
            text-align: center;\
            font-family: 'Courier New', Courier, monospace;\
            font-size: 28px;\
            cursor: pointer;\
        }\
    </style>\
</head>\
<body>\
    <h1>INDACT Robotkar</h1>\
    <div class=\"status\">\
        <b>&Aacute;llapot: </b>";

static const char control_page_after_status[] PROGMEM =
"    </div>\
    <form method=\"get\">\
        <table class=\"controls\">\
            <tr>\
                <th>z ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"zp\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"zm\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>&phi; ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"fp\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"fm\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>r ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"rp\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"rm\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>Alaphelyzet</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"ho\">H</button>\
                </td>\
            </tr>\
        </table>\
    </form>\
</body>\
</html>";

static const char back_to_index_page[] PROGMEM =
"<!DOCTYPE html>\
<html>\
<head>\
    <title>INDACT Robotkar</title>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <style>\
        div {\
            margin: 10px;\
            padding: 10px;\
            background-color: lightgray;\
            text-align: center;\
            font-size: 20px;\
        }\
        a {\
            color: blueviolet;\
            font-size: 24px;\
        }\
    </style>\
</head>\
<body>\
    <div>\
        <h1>Hiba!</h1>\
        <p>Az elk&uuml;ld&ouml;tt k&eacute;r&eacute;s hib&aacute;s vagy nem siker&uuml;lt felismerni.</p>\
        <p><a href=\"/\">Vissza a vez&eacute;rl&odblac; oldalra</a></p>\
    </div>\
</body>\
</html>";
