"""Every extra subscribes on a Klipper without mcu.register_serial_response (Snapmaker's U1
fork) exactly as on the pinned Klipper: the same report names, nothing more, nothing less."""

from test_ace2k_dryer import DRYER_RESPONSES
from test_ace2k_extras import RESPONSES, FakeMcu, LegacyFakeMcu, make
from test_ace2k_rfid import make_rfid


def names(mcu):
    return set(mcu.subscriptions)


def test_ace2k_and_the_feed_subscribe_the_same_reports_on_a_legacy_klipper():
    _, _, new = make(RESPONSES)
    new.config_callback()
    _, _, old = make(RESPONSES, mcu_class=LegacyFakeMcu)
    old.config_callback()
    assert names(old) == names(new)
    assert "ace2k_feed_state" in names(old)


def test_the_dryer_subscribes_the_same_reports_on_a_legacy_klipper():
    _, _, new = make(dict(DRYER_RESPONSES))
    new.config_callback()
    _, _, old = make(dict(DRYER_RESPONSES), mcu_class=LegacyFakeMcu)
    old.config_callback()
    assert names(old) == names(new)
    assert "ace2k_dryer_state" in names(old)


def test_the_rfid_subscribes_the_same_reports_on_a_legacy_klipper():
    _, _, new = make_rfid()
    _, _, old = make_rfid(mcu_class=LegacyFakeMcu)
    assert names(old) == names(new)
    assert "ace2k_rfid_state" in names(old)


def test_the_legacy_fake_has_no_newer_method():
    assert LegacyFakeMcu(RESPONSES).register_serial_response is None
    assert FakeMcu(RESPONSES).register_serial_response is not None
