//
// Created by clemens on 02.07.22.
//

#ifndef SRC_XESC_INTERFACE_H
#define SRC_XESC_INTERFACE_H

#include <xesc_msgs/XescStateStamped.h>


namespace xesc_interface {
    class XescInterface {
    public:
        virtual void getStatus(xesc_msgs::XescStateStamped &state)=0;
        virtual void getStatusBlocking(xesc_msgs::XescStateStamped &state)=0;
        virtual void setDutyCycle(float duty_cycle)=0;
        // Closed-loop speed command in electrical RPM (ERPM). Backends that cannot do ERPM
        // control must refuse safely (see implementations), never silently no-op or throw.
        virtual void setSpeed(float erpm)=0;
        virtual void stop()=0;

    };
}

#endif
