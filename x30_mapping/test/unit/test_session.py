"""Mapping sessions: the grid's site (#4) and the frame rules of mapping/start (#9, #13)."""

from __future__ import annotations

from x30_mapping.core.session import (
    Localization,
    LocState,
    MappingSession,
    SessionState,
    StartPlan,
    plan_start,
)

NONE = Localization()
ORIGIN = Localization(LocState.LOCALIZED, 'origin')
TAG = Localization(LocState.LOCALIZED, 'tag:base_map')
ESTIMATE = Localization(LocState.LOCALIZED, 'pose_estimate')
UNCERTAIN_TAG = Localization(LocState.UNCERTAIN, 'tag:base_map')


def test_session_refuses_to_save_another_sites_grid():
    """#4: the grid remembers its site; a save into another site is refused."""
    session = MappingSession()
    assert session.check_save('lobby') == 'nothing mapped yet'
    assert session.start('lobby', reset=True) == ''
    assert session.check_save('lobby') == ''
    assert 'refusing' in session.check_save('hall')
    assert 'already mapping' in session.start('hall', reset=True)
    session.stop()
    assert session.state is SessionState.STOPPED
    assert 'refusing' in session.check_save('hall')
    assert 'load hall first' in session.start('hall', reset=False)
    assert session.load('hall') == ''
    assert session.start('hall', reset=False) == ''
    assert 'stop first' in session.load('lobby')


def test_origin_at_boot_is_not_the_frame_of_an_existing_site():
    """#9: continuing a site with content needs a tag or a pose estimate, not 'origin'."""
    plan = plan_start('lobby', False, 'lobby', True, True, ORIGIN)
    assert plan.localize and plan.resume and not plan.archive
    assert plan_start('lobby', False, 'lobby', True, True, TAG) == StartPlan(resume=True)
    assert plan_start('lobby', False, 'lobby', True, True, ESTIMATE) == StartPlan(resume=True)
    assert plan_start('lobby', False, 'lobby', True, True, UNCERTAIN_TAG).localize


def test_frame_adopted_in_this_run_counts_for_its_site():
    """A new site mapped from 'origin' can be continued while that localization lasts."""
    assert plan_start('lobby', False, 'lobby', True, True, ORIGIN, 'lobby') == StartPlan(
        resume=True
    )
    assert plan_start('lobby', False, 'lobby', True, True, NONE, 'lobby').localize
    assert plan_start('hall', False, 'lobby', True, False, ORIGIN, 'lobby').localize


def test_another_site_resets_localization_first():
    plan = plan_start('hall', False, 'lobby', True, False, TAG)
    assert plan.activate and plan.reset_localization and plan.localize and not plan.resume


def test_reset_always_archives_existing_content():
    """#13: reset archives the old map even on the active site in its own frame."""
    plan = plan_start('lobby', True, 'lobby', True, True, TAG)
    assert plan.archive and plan.activate and not plan.resume and not plan.assume_origin
    assert not plan_start('lobby', True, 'lobby', False, False, TAG).archive
    assert plan_start('lobby', True, '', True, True, NONE).assume_origin


def test_new_site_takes_the_current_frame_or_the_origin():
    assert plan_start('new', False, 'lobby', False, False, TAG) == StartPlan(activate=True)
    assert plan_start('new', False, '', False, False, NONE) == StartPlan(
        activate=True, assume_origin=True
    )
