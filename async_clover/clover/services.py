from .. ros_wrappers import AsyncService, AsyncSubscriber


from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState


class CloverFlightServices:
    get_telemetry: AsyncService[srv.GetTelemetry] = AsyncService("get_telemetry", srv.GetTelemetry)
    navigate: AsyncService[srv.Navigate] = AsyncService("navigate", srv.Navigate)
    navigate_global: AsyncService[srv.NavigateGlobal] = AsyncService("navigate_global", srv.NavigateGlobal)
    set_position: AsyncService[srv.SetPosition] = AsyncService("set_position", srv.SetPosition)
    set_velocity: AsyncService[srv.SetVelocity] = AsyncService("set_velocity", srv.SetVelocity)
    set_attitude: AsyncService[srv.SetAttitude] = AsyncService("set_attitude", srv.SetAttitude)
    set_rates: AsyncService[srv.SetRates] = AsyncService("set_rates", srv.SetRates)

    land: AsyncService[Trigger] = AsyncService("land", Trigger)
    arming: AsyncService[CommandBool] = AsyncService("mavros/cmd/arming", CommandBool)


class CloverLedServices:
    set_effect: AsyncService[srv.SetLEDEffect] = AsyncService("led/set_effect", srv.SetLEDEffect)
    set_leds: AsyncService[SetLEDs] = AsyncService("led/set_leds", SetLEDs)
