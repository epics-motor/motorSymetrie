import pytest
from epics import PV, caget, caput
from time import sleep


class Controller:
    def __init__(self):
        self._prefix = 'SYM:HEX01:'
        self._raw = PV('{}TERMINAL:String'.format(self._prefix))
        self._raw_RBV = PV('{}TERMINAL:String_RBV'.format(self._prefix))

    def turn_on(self):
        # Turn control on
        self.put('CONTROLON.PROC', 1)
        sleep(1)
        # Home the device
        self.put('HOME.PROC', 1)
        sleep(1)

    def write_read(self, cmd):
        self._raw.put(cmd)
        return self._raw_RBV.get()

    def put(self, pv, value):
        caput('{}{}'.format(self._prefix, pv), value)

    def get(self, pv):
        return caget('{}{}'.format(self._prefix, pv))


@pytest.fixture
def controller():
    return Controller()


class TestMainFunctionality:

    def test_control(self, controller):
        # Turn off the controller
        controller.put('CONTROLOFF.PROC', 1)
        sleep(1)
        # Check that the controller has turned off
        assert(controller.get('s_hexa:ControlOn_RBV') == 0)
        # Turn on the controller
        controller.put('CONTROLON.PROC', 1)
        sleep(1)
        # Check that the controller has turned on
        assert(controller.get('s_hexa:ControlOn_RBV') == 1)

    def test_error(self, controller):
        # Turn on
        controller.turn_on()
        # Force an error in axes
        controller.write_read('P_Ax_Err_EncoderError(1)=1')
        controller.write_read('P_Ax_Err_EncoderError(2)=1')
        controller.write_read('P_Ax_Err_EncoderError(3)=1')
        sleep(3)
        # Verify the error flag is set
        assert(controller.get('s_hexa:Error_RBV') > 0)
        # Verify non zero error codes are present
        assert(controller.get('ERR_INFO:0:Code') != 0)
        assert(controller.get('ERR_INFO:1:Code') != 0)
        # Clear the error
        controller.write_read('P_Ax_Err_EncoderError(1)=0')
        controller.write_read('P_Ax_Err_EncoderError(2)=0')
        controller.write_read('P_Ax_Err_EncoderError(3)=0')
        sleep(1)
        controller.put('CLEARERROR.PROC', 1)
        sleep(2)
        # Verify the error flag is clear
        assert(controller.get('s_hexa:Error_RBV') == 0)
        # Verify error codes are clear
        assert(controller.get('ERR_INFO:0:Code') == 0)
        assert(controller.get('ERR_INFO:1:Code') == 0)

    def test_stop(self, controller):
        # Turn on
        controller.turn_on()
        # Move to USER_ZERO
        controller.put('MOVE_SPECIFICPOS:Index', 'USER ZERO')
        controller.put('MOVE_SPECIFICPOS.PROC', 1)
        sleep(2)
        # Wait for In position
        counter = 0
        while controller.get('s_hexa:InPosition_RBV') == 0 and counter < 100:
            counter += 1
            sleep(1)

        # Start moving to position far away
        controller.put('MOVE_PTP:Tx', 0.0)
        controller.put('MOVE_PTP:Ty', 0.0)
        controller.put('MOVE_PTP:Tz', -20.0)
        controller.put('MOVE_PTP:Rx', 0.0)
        controller.put('MOVE_PTP:Ry', 0.0)
        controller.put('MOVE_PTP:Rz', 0.0)
        controller.put('MOVE_PTP.PROC', 1)
        # Wait for a few seconds
        sleep(5)
        # Press the stop button
        controller.put('STOP.PROC', 1)
        sleep(3)
        # Verfiy in position is true
        assert(controller.get('s_hexa:InPosition_RBV') == 1)
        # Verfiy position is not equal to setpoint
        assert(controller.get('s_uto_tz_RBV') != pytest.approx(-20.0, abs=0.001))

    def test_home(self, controller):
        # Force an error that takes the controller out of home valid
        controller.write_read('P_Ax_Err_EncoderError(2)=1')
        sleep(1)
        controller.write_read('P_Ax_Err_EncoderError(2)=0')
        sleep(1)
        # Clear the error 
        controller.put('CLEARERROR.PROC', 1)
        sleep(1)
        # Check home complete is false
        assert(controller.get('s_hexa:HomeComplete_RBV') == 0)
        # Turn control on
        controller.put('CONTROLON.PROC', 1)
        sleep(1)
        # Home the device
        controller.put('HOME.PROC', 1)
        sleep(1)
        # Check home complete is false
        assert(controller.get('s_hexa:HomeComplete_RBV') == 1)

    def test_move_specific(self, controller):
        # Turn on
        controller.turn_on()
        # Move to USER_ZERO
        controller.put('MOVE_SPECIFICPOS:Index', 'USER ZERO')
        controller.put('MOVE_SPECIFICPOS.PROC', 1)
        sleep(2)
        # Wait for In position
        counter = 0
        while controller.get('s_hexa:InPosition_RBV') == 0 and counter < 100:
            counter += 1
            sleep(1)

        # Verify the in position is true and the axes are at zero
        sleep(1)
        assert(controller.get('s_hexa:InPosition_RBV') == 1)
        assert(controller.get('s_uto_tx_RBV') == pytest.approx(0.0, abs=0.001))
        assert(controller.get('s_uto_ty_RBV') == pytest.approx(0.0, abs=0.001))
        assert(controller.get('s_uto_tz_RBV') == pytest.approx(0.0, abs=0.001))
        assert(controller.get('s_uto_rx_RBV') == pytest.approx(0.0, abs=0.001))
        assert(controller.get('s_uto_ry_RBV') == pytest.approx(0.0, abs=0.001))
        assert(controller.get('s_uto_rz_RBV') == pytest.approx(0.0, abs=0.001))

    def test_validate(self, controller):
        # Turn on
        controller.turn_on()
        # Reset all demand positions to zero
        controller.put('MOVE_PTP:Tx', 0.0)
        controller.put('MOVE_PTP:Ty', 0.0)
        controller.put('MOVE_PTP:Tz', 0.0)
        controller.put('MOVE_PTP:Rx', 0.0)
        controller.put('MOVE_PTP:Ry', 0.0)
        controller.put('MOVE_PTP:Rz', 0.0)
        sleep(2)
        # Verify Valid is good
        assert(controller.get('VALID_PTP:Result_RBV') == 0)
        # Change each demand and verify valid goes bad if outside range
        controller.put('MOVE_PTP:Tx', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Tx', 0.0)
        sleep(2)

        controller.put('MOVE_PTP:Ty', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Ty', 0.0)
        sleep(2)

        controller.put('MOVE_PTP:Tz', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Tz', 0.0)
        sleep(2)

        controller.put('MOVE_PTP:Rx', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Rx', 0.0)
        sleep(2)

        controller.put('MOVE_PTP:Ry', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Ry', 0.0)
        sleep(2)

        controller.put('MOVE_PTP:Rz', 10000.0)
        sleep(2)
        assert(controller.get('VALID_PTP:Result_RBV') > 0)
        controller.put('MOVE_PTP:Rz', 0.0)
        sleep(2)

    def test_move_ptp(self, controller):
        # Turn on
        controller.turn_on()
        # Move to USER_ZERO
        controller.put('MOVE_SPECIFICPOS:Index', 'USER ZERO')
        controller.put('MOVE_SPECIFICPOS.PROC', 1)
        sleep(2)
        # Wait for In position
        counter = 0
        while controller.get('s_hexa:InPosition_RBV') == 0 and counter < 100:
            counter += 1
            sleep(1)

        # Move to demand position
        controller.put('MOVE_PTP:Tx', 1.0)
        controller.put('MOVE_PTP:Ty', 2.0)
        controller.put('MOVE_PTP:Tz', 3.0)
        controller.put('MOVE_PTP:Rx', 4.0)
        controller.put('MOVE_PTP:Ry', 5.0)
        controller.put('MOVE_PTP:Rz', 6.0)
        controller.put('MOVE_PTP:MoveType', 'ABSOLUTE')
        sleep(2)
        controller.put('MOVE_PTP.PROC', 1)
        sleep(2)
        # Wait for In position
        counter = 0
        while controller.get('s_hexa:InPosition_RBV') == 0 and counter < 100:
            counter += 1
            sleep(1)

        # Verify the in position is true and the axes are at position
        sleep(1)
        assert(controller.get('s_hexa:InPosition_RBV') == 1)
        assert(controller.get('s_uto_tx_RBV') == pytest.approx(1.0, abs=0.001))
        assert(controller.get('s_uto_ty_RBV') == pytest.approx(2.0, abs=0.001))
        assert(controller.get('s_uto_tz_RBV') == pytest.approx(3.0, abs=0.001))
        assert(controller.get('s_uto_rx_RBV') == pytest.approx(4.0, abs=0.001))
        assert(controller.get('s_uto_ry_RBV') == pytest.approx(5.0, abs=0.001))
        assert(controller.get('s_uto_rz_RBV') == pytest.approx(6.0, abs=0.001))

