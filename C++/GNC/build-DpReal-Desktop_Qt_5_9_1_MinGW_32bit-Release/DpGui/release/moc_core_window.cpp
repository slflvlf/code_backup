/****************************************************************************
** Meta object code from reading C++ file 'core_window.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../DPreal/DpGui/core_window.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'core_window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CoreWindow_t {
    QByteArrayData data[9];
    char stringdata0[103];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CoreWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CoreWindow_t qt_meta_stringdata_CoreWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "CoreWindow"
QT_MOC_LITERAL(1, 11, 11), // "poseChanged"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 15), // "QVector<double>"
QT_MOC_LITERAL(4, 40, 15), // "setpointChanged"
QT_MOC_LITERAL(5, 56, 11), // "servo_close"
QT_MOC_LITERAL(6, 68, 15), // "messageReceived"
QT_MOC_LITERAL(7, 84, 3), // "msg"
QT_MOC_LITERAL(8, 88, 14) // "setpointSelect"

    },
    "CoreWindow\0poseChanged\0\0QVector<double>\0"
    "setpointChanged\0servo_close\0messageReceived\0"
    "msg\0setpointSelect"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CoreWindow[] = {

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
       4,    1,   42,    2, 0x06 /* Public */,
       5,    0,   45,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   46,    2, 0x0a /* Public */,
       8,    0,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    7,
    QMetaType::Void,

       0        // eod
};

void CoreWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CoreWindow *_t = static_cast<CoreWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->poseChanged((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 1: _t->setpointChanged((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 2: _t->servo_close(); break;
        case 3: _t->messageReceived((*reinterpret_cast< const QVector<double>(*)>(_a[1]))); break;
        case 4: _t->setpointSelect(); break;
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
            typedef void (CoreWindow::*_t)(const QVector<double> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CoreWindow::poseChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (CoreWindow::*_t)(const QVector<double> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CoreWindow::setpointChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (CoreWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CoreWindow::servo_close)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject CoreWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CoreWindow.data,
      qt_meta_data_CoreWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *CoreWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CoreWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CoreWindow.stringdata0))
        return static_cast<void*>(const_cast< CoreWindow*>(this));
    return QWidget::qt_metacast(_clname);
}

int CoreWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void CoreWindow::poseChanged(const QVector<double> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CoreWindow::setpointChanged(const QVector<double> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CoreWindow::servo_close()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
