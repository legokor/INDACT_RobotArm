/**
 *******************************************************************************
 * @file robotkar_gui_html.h
 *
 * @date Jan 20, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This file contains the HTML pages used by the webserver.
 *******************************************************************************
 */

#ifndef ROBOTKAR_GUI_HTML_H_
#define ROBOTKAR_GUI_HTML_H_

static const char control_page_section_1[] PROGMEM =
  "<!DOCTYPE html>\
<html>\
<head>\
    <title>INDACT Robotkar</title>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <style>\
        div {\
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
        button {\
            border: 1px solid black;\
            border-radius: 8px;\
            color: black;\
            width: 80px;\
            margin: 4px;\
            padding: 10px;\
            text-align: center;\
            font-family: 'Courier New', Courier, monospace;\
            font-size: 28px;\
            cursor: pointer;\
        }\
        button:hover {\
            background-color: darkgray;\
            color: white;\
        }\
    </style>\
</head>\
<body>\
    <h1>INDACT Robotkar</h1>\
    <div id=\"status\">\
        <b>&Aacute;llapot: </b>";  // insert status here

static const char control_page_section_2[] PROGMEM =
  "    </div>\
    <form method=\"get\">\
        <table class=\"controls\">\
            <tr>\
                <th>";  // insert 1. axis name here

static const char control_page_section_3[] PROGMEM =
  " ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"ap\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"am\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>";  // insert 2. axis name here

static const char control_page_section_4[] PROGMEM =
  " ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"bp\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"bm\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>";  // insert 3. axis name here

static const char control_page_section_5[] PROGMEM =
  " ir&aacute;ny</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"cp\">+</button><br>\
                    <button type=\"submit\" name=\"mov\" value=\"cm\">-</button>\
                </td>\
            </tr>\
            <tr>\
                <th>Alaphelyzet</th>\
                <td>\
                    <button type=\"submit\" name=\"mov\" value=\"hx\">H</button>\
                </td>\
            </tr>\
        </table>\
    </form>\
    <form method=\"get\">\
        <div id=\"coordinates\">\
            <b>Koordin&aacute;ta rendszer v&aacute;lt&aacute;s:</b>\
            (r, &phi;, z) vagy (x, y, z)\
            <button type=\"submit\" name=\"mov\" value=\"kx\">C</button>\
        </div>\
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

static const char wait_for_change_page[] PROGMEM =
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
        <h1>Vez&eacute;rl&odblac; friss&iacute;tve</h1>\
        <p><a href=\"/\">Vissza a vez&eacute;rl&odblac; oldalra</a></p>\
    </div>\
</body>\
</html>";

#endif /* ROBOTKAR_GUI_HTML_H_ */
