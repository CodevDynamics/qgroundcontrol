#include "AVIATORInterface.h"
#include "QGCLoggingCategory.h"
#include <QQmlEngine>
#include <QString>
#include "QGCApplication.h"

QGC_LOGGING_CATEGORY(AVIATORInterfaceLog, "AVIATORInterfaceLog")

const char* AVIATORInterface::_batteryVoltageFactName = "RC_BAT_VOLTAGE";
const char* AVIATORInterface::_batteryRemainingFactName = "RC_BAT_REMAINING";
const char* AVIATORInterface::_versionFactName = "RC_VERSION";
const char* AVIATORInterface::_temperatureFactName = "RC_TEMPERATURE";
const char* AVIATORInterface::_usbOutFactName = "RC_USB_OUT";
const char* AVIATORInterface::_batteryCurrentFactName = "RC_BAT_CURRENT";
const char* AVIATORInterface::_batteryChargingFactName = "RC_BAT_CHARGING";

AVIATORInterface::AVIATORInterface(QObject* parent)
    : FactGroup(1000, ":/json/AVIATORFact.json", parent)
    , _batteryVoltageFact(0, _batteryVoltageFactName, FactMetaData::valueTypeFloat)
    , _batteryRemainingFact(0, _batteryRemainingFactName, FactMetaData::valueTypeFloat)
    , _versionFact(0, _versionFactName, FactMetaData::valueTypeFloat)
    , _temperatureFact(0, _temperatureFactName, FactMetaData::valueTypeFloat)
    , _usbOutFact(0, _usbOutFactName, FactMetaData::valueTypeFloat)
    , _batteryCurrentFact(0, _batteryCurrentFactName, FactMetaData::valueTypeFloat)
    , _batteryChargingFact(0, _batteryChargingFactName, FactMetaData::valueTypeFloat)
#ifdef Q_OS_ANDROID
    , _portName("/dev/ttysWK0")
#endif
    , _baudRate(115200)
{
    qmlRegisterUncreatableType<AVIATORInterface>("QGroundControl", 1, 0, "AVIATORInterface", "Reference only");

    _addFact(&_batteryVoltageFact, _batteryVoltageFactName);
    _addFact(&_batteryRemainingFact, _batteryRemainingFactName);
    _addFact(&_versionFact, _versionFactName);
    _addFact(&_temperatureFact, _temperatureFactName);
    _addFact(&_usbOutFact, _usbOutFactName);
    _addFact(&_batteryCurrentFact, _batteryCurrentFactName);
    _addFact(&_batteryChargingFact, _batteryChargingFactName);

#if defined (Q_OS_ANDROID)
    QObject::connect(this, &AVIATORInterface::bytesReceived, this, &AVIATORInterface::_handlebytesReceived);
    QObject::connect(this, &AVIATORInterface::write, this, &AVIATORInterface::_writeBytes);
    _init();
#endif
}

AVIATORInterface::~AVIATORInterface()
{
    if (_port) {
        // This prevents stale signals from calling the link after it has been deleted
        QObject::disconnect(_port, &QIODevice::readyRead, this, &AVIATORInterface::_readBytes);
        _port->close();
        _port->deleteLater();
        _port = nullptr;
    }
}

void AVIATORInterface::_writeBytes(const QByteArray data)
{
    if(_port && _port->isOpen()) {
        _port->write(data);
    } else {
        // Error occurred
        qWarning() << "Serial port not writeable" << _portName;
    }
}

void AVIATORInterface::_readBytes()
{
    if (_port && _port->isOpen()) {
        qint64 byteCount = _port->bytesAvailable();
        if (byteCount) {
            QByteArray buffer;
            buffer.resize(byteCount);
            _port->read(buffer.data(), buffer.size());
            for (int position = 0; position < buffer.size(); position++) {
                mavlink_message_t message;
                mavlink_status_t status;
                if (mavlink_parse_char(MAVLINK_AVIATOR_COMM_ID, static_cast<uint8_t>(buffer[position]), &message, &status)) {
                    emit bytesReceived(message);
                }
            }
        }
    } else {
        // Error occurred
        qWarning() << "Serial port not readable" << _portName;
    }
}

void AVIATORInterface::_init()
{
    if(_port) {
        qCDebug(AVIATORInterfaceLog) << QString::number((qulonglong)this, 16) << "closing port";
        _port->close();

        delete _port;
        _port = nullptr;
    }

    qCDebug(AVIATORInterfaceLog) << "init " << _portName;
    _port = new QSerialPort(_portName, this);
    // QObject::connect(_port, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error), this, &AVIATORInterface::linkError);
    QObject::connect(_port, &QIODevice::readyRead, this, &AVIATORInterface::_readBytes);
    _port->open(QIODevice::ReadWrite);
    if (!_port->isOpen() ) {
        qWarning() << "open failed" << _port->errorString() << _port->error() << _portName;
        _port->close();
        delete _port;
        _port = nullptr;
    } else {
        _port->setDataTerminalReady(true);
        _port->setBaudRate(_baudRate);
    }
}

void AVIATORInterface::_handlebytesReceived(const mavlink_message_t& message)
{
    switch(message.msgid) {
    case MAVLINK_MSG_ID_RC_CHANNELS:
        _handle_mavlink_rc_channels(message);
        break;
    case MAVLINK_MSG_ID_PARAM_VALUE:
        _handle_mavlink_param_value(message);
        break;
    default:
        break;
    }
}

void AVIATORInterface::_handle_mavlink_rc_channels(const mavlink_message_t& message)
{
    _rcChannelValues.clear();
    mavlink_rc_channels_t channels;
    mavlink_msg_rc_channels_decode(&message, &channels);

    _rcChannelValues.append(channels.chan1_raw);
    _rcChannelValues.append(channels.chan2_raw);
    _rcChannelValues.append(channels.chan3_raw);
    _rcChannelValues.append(channels.chan4_raw);
    _rcChannelValues.append(channels.chan5_raw);
    _rcChannelValues.append(channels.chan6_raw);
    _rcChannelValues.append(channels.chan7_raw);
    _rcChannelValues.append(channels.chan8_raw);
    _rcChannelValues.append(channels.chan9_raw);
    _rcChannelValues.append(channels.chan10_raw);
    _rcChannelValues.append(channels.chan11_raw);
    _rcChannelValues.append(channels.chan12_raw);
    _rcChannelValues.append(channels.chan13_raw);
    _rcChannelValues.append(channels.chan14_raw);
    _rcChannelValues.append(channels.chan15_raw);
    _rcChannelValues.append(channels.chan16_raw);
    _rcChannelValues.append(channels.chan17_raw);
    _rcChannelValues.append(channels.chan18_raw);

    channels.chancount = 18;
    memset(_rawChannels, 0xff, 18 * 2);
    memcpy(_rawChannels, &channels.chan1_raw, channels.chancount * 2);

    emit rcChannelValuesChanged(_rawChannels, channels.chancount);

    bool f1 = channels.chan15_raw == 2000;
    bool f2 = channels.chan14_raw == 2000;
    bool f3 = channels.chan16_raw == 2000;
    bool capture = channels.chan12_raw == 2000;
    bool record = channels.chan13_raw == 2000;

    if(f1 != _f1Pressed) {
        _f1Pressed = f1;
        emit buttonPressed(AVIATOR_FUNCTION_GIMBAL_RESET, _f1Pressed);
    }

    if(f2 != _f2Pressed) {
        _f2Pressed = f2;
        emit buttonPressed(AVIATOR_FUNCTION_THERMAL_ZOOM, _f2Pressed);
    }

    static int f3Count = 0;
    if(f3) f3Count++;
    else {
        if(f3Count > 0 && f3Count < 50) { // 1s
            emit buttonPressed(AVIATOR_FUNCTION_IR_SWITCH, true);
        }
        f3Count = 0;
    }
    bool f3Pressed = (f3Count > 250); // 5s
    if(f3Pressed != _f3Pressed) {
        _f3Pressed = f3Pressed;
    }

    if(capture != _capturePressed) {
        _capturePressed = capture;
        emit buttonPressed(AVIATOR_FUNCTION_CAMERA_CAPTURE, _capturePressed);
    }

    if(record != _recordPressed) {
        _recordPressed = record;
        emit buttonPressed(AVIATOR_FUNCTION_CAMERA_TOGGLE_RECORD, _recordPressed);
    }
}

void AVIATORInterface::_handle_mavlink_param_value(const mavlink_message_t& message)
{
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&message, &param_value);

    // This will null terminate the name string
    QByteArray bytes(param_value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    QString parameterName(bytes);

    Fact* fact = getFact(parameterName);

    if(fact) {
        mavlink_param_union_t paramUnion;
        paramUnion.param_float = param_value.param_value;
        paramUnion.type = param_value.param_type;

        QVariant parameterValue;

        switch (paramUnion.type) {
        case MAV_PARAM_TYPE_REAL32:
            parameterValue = QVariant(paramUnion.param_float);
            break;
        case MAV_PARAM_TYPE_UINT8:
            parameterValue = QVariant(paramUnion.param_uint8);
            break;
        case MAV_PARAM_TYPE_INT8:
            parameterValue = QVariant(paramUnion.param_int8);
            break;
        case MAV_PARAM_TYPE_UINT16:
            parameterValue = QVariant(paramUnion.param_uint16);
            break;
        case MAV_PARAM_TYPE_INT16:
            parameterValue = QVariant(paramUnion.param_int16);
            break;
        case MAV_PARAM_TYPE_UINT32:
            parameterValue = QVariant(paramUnion.param_uint32);
            break;
        case MAV_PARAM_TYPE_INT32:
            parameterValue = QVariant(paramUnion.param_int32);
            break;
        default:
            qCritical() << "AVIATORInterface::_handle_mavlink_param_value - unsupported MAV_PARAM_TYPE" << paramUnion.type;
            break;
        }

        fact->setRawValue(parameterValue);
    }
}
