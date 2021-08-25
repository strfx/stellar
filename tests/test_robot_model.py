"""
Test `Robot` model.
"""
import pytest
import numpy as np
from pytest import fixture
from stellar.models.robot import Robot

from hypothesis import given
import hypothesis.strategies as some


@fixture(scope="module")
def robot():
    return Robot()


def test_robot_initializes_at_origin(robot):
    assert robot.x == robot.y == robot.theta == 0.0


def test_is_in_goal():
    robot = Robot()
    robot.set(2, 2, np.radians(90))

    goal = (2, 2)
    assert robot.in_goal(goal, proximity=0.0)

    goal = (1.8, 1.8)
    assert robot.in_goal(goal, proximity=0.5)

    goal = (1.8, 1.8)
    assert not robot.in_goal(goal, proximity=0.0)


def test_in_goal_respects_min_distance():
    """
    Must take min_distance into account for the goal check.
    Otherwise, it might signal that the robot has reached its goal
    before actually driving somewhere.
    """
    robot = Robot()
    robot.set(0, 0, np.pi / 2)
    robot.move(1, 0)
    robot.move(1, 0)
    assert pytest.approx(robot.y, 0.01) == 2
    assert pytest.approx(robot.x, 0.01) == 0
    goal = (0, 2)
    assert robot.in_goal(goal, proximity=0.1, min_distance=1)
    assert not robot.in_goal(goal, proximity=0.1, min_distance=4)


@given(some.floats(min_value=-4 * np.pi, max_value=4 * np.pi))
def test_robots_orientation_is_in_range(robot, steer):
    robot.move(1, steer)
    assert -2 * np.pi <= robot.theta <= 2 * np.pi


@given(some.integers())
def test_robot_forward_motion(distance):
    robot = Robot()
    robot.move(distance, 0)
    assert robot.x >= 0.0

    if distance >= 0.0:
        assert pytest.approx(robot.x, 0.1) == distance


@given(some.integers())
def test_robot_backward_motion(distance):
    robot = Robot()
    # Set orientation to 180°
    robot.set(0, 0, np.pi)
    robot.move(distance, 0)
    assert robot.x <= 0.0
    if distance > 0.0:
        assert pytest.approx(robot.x, 0.1) == -distance


@given(some.integers())
def test_robot_upwards_motion(distance):
    robot = Robot()
    robot.move(distance, np.pi / 2)
    assert robot.y >= 0.0
    if distance >= 0.0:
        assert pytest.approx(robot.y, 0.1) == distance


@given(some.integers())
def test_robot_downwards_motion(distance):
    robot = Robot()
    steer = -np.pi / 2
    robot.move(distance, steer)
    assert robot.y <= 0.0

    if distance >= 0.0:
        assert pytest.approx(robot.y, 0.1) == -distance
