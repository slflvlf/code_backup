/****************************************************************************
** Meta object code from reading C++ file 'plant.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../DpSim/plant.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'plant.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Plant_t {
    QByteArrayData data[9];
    char stringdata0[96];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Plant_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Plant_t qt_meta_stringdata_Plant = {
    {
QT_MOC_LITERAL(0, 0, 5), // "Plant"
QT_MOC_LITERAL(1, 6, 12), // "messageReady"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 15), // "QVector<double>"
QT_MOC_LITERAL(4, 36, 3), // "msg"
QT_MOC_LITERAL(5, 40, 15), // "messageReceived"
QT_MOC_LITERAL(6, 56, 15), // "abortSimulation"
QT_MOC_LITERAL(7, 72, 13), // "sendActuation"
QT_MOC_LITERAL(8, 86, 9) // "sendAbort"

    },
    "Plant\0messageReady\0\0QVector<double>\0"
    "msg\0messageReceived\0abortSimulation\0"
    "sendActuation\0sendAbort"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Plant[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,
       6,    0,   45,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   46,    2, 0x0a /* Public */,
       8,    0,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

       0        // eod
};

void Plant::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Plant *_t = static_cast<Plant *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->messageReady((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 1: _t->messageReceived((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 2: _t->abortSimulation(); break;
        case 3: _t->sendActuation((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 4: _t->sendAbort(); break;
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
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<double> >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<double> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Plant::*_t)(const QVector<double> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Plant::messageReady)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (Plant::*_t)(const QVector<double> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Plant::messageReceived)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (Plant::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Plant::abortSimulation)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject Plant::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Plant.data,
      qt_meta_data_Plant,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Plant::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Plant::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Plant.stringdata0))
        return static_cast<void*>(const_cast< Plant*>(this));
    return QObject::qt_metacast(_clname);
}

int Plant::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void Plant::messageReady(const QVector<double> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Plant::messageReceived(const QVector<double> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Plant::abortSimulation()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
