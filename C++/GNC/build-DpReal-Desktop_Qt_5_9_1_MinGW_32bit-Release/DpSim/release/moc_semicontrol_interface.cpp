/****************************************************************************
** Meta object code from reading C++ file 'semicontrol_interface.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../DPreal/DpSim/semicontrol_interface.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'semicontrol_interface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Semicontrol_interface_t {
    QByteArrayData data[6];
    char stringdata0[72];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Semicontrol_interface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Semicontrol_interface_t qt_meta_stringdata_Semicontrol_interface = {
    {
QT_MOC_LITERAL(0, 0, 21), // "Semicontrol_interface"
QT_MOC_LITERAL(1, 22, 16), // "thrusterReceived"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 15), // "QVector<double>"
QT_MOC_LITERAL(4, 56, 3), // "msg"
QT_MOC_LITERAL(5, 60, 11) // "servo_close"

    },
    "Semicontrol_interface\0thrusterReceived\0"
    "\0QVector<double>\0msg\0servo_close"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Semicontrol_interface[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x0a /* Public */,
       5,    0,   27,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

       0        // eod
};

void Semicontrol_interface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Semicontrol_interface *_t = static_cast<Semicontrol_interface *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->thrusterReceived((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 1: _t->servo_close(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<double> >(); break;
            }
            break;
        }
    }
}

const QMetaObject Semicontrol_interface::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Semicontrol_interface.data,
      qt_meta_data_Semicontrol_interface,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Semicontrol_interface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Semicontrol_interface::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Semicontrol_interface.stringdata0))
        return static_cast<void*>(const_cast< Semicontrol_interface*>(this));
    return QObject::qt_metacast(_clname);
}

int Semicontrol_interface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
