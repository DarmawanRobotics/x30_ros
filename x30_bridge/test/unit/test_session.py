"""One handheld at a time: the token, taking over, acknowledgements."""

from __future__ import annotations

from x30_bridge.core.session import MAX_ACKS, Session

A, A2, B = ('10.0.0.2', 4000), ('10.0.0.2', 4001), ('10.0.0.3', 4000)


def test_the_token_is_checked():
    assert Session('secret', 2.0).authorized({'k': 'secret'})
    assert not Session('secret', 2.0).authorized({'k': 'wrong'})
    assert not Session('secret', 2.0).authorized({})
    assert Session('', 2.0).authorized({})


def test_one_device_drives_until_it_falls_silent():
    session = Session('', 2.0)
    assert session.accept(A, 'h12', 0.0)
    assert session.busy_for(B, 1.0) == 'robot is driven from 10.0.0.2'
    assert session.busy_for(A2, 1.0) == ''  # the same device after an app restart
    assert session.busy_for(B, 3.0) == ''  # the first one went silent
    assert not session.accept(A, 'h12', 1.0)  # the same client: no new session
    number = session.number
    assert session.accept(A2, 'h12', 1.5) and session.number == number + 1


def test_a_new_session_forgets_the_acks_and_keeps_few():
    session = Session('', 2.0)
    session.accept(A, '', 0.0)
    session.remember((1, 'estop'), {'ok': True})
    session.accept(A2, '', 0.1)
    assert session.acks == {}
    for i in range(MAX_ACKS + 5):
        session.remember((i, 'x'), {})
    assert len(session.acks) == MAX_ACKS


def test_denied_at_most_once_a_second_and_an_end():
    session = Session('secret', 2.0)
    assert session.deny_now(A, 0.0)
    assert not session.deny_now(A, 0.5)
    assert session.deny_now(B, 0.5) and session.deny_now(A, 1.1)
    assert not session.end()
    session.accept(A, '', 0.0)
    assert session.end() and session.client is None and not session.alive(0.0)
