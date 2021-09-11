"""
__init__.py: Definitions for the occupancy grid.

Created by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from enum import Enum
from json import dumps, loads
from typing import NamedTuple

import numpy as np

from .const import (UPDATE_ACTION, UPDATE_ACTION_LEAVE, UPDATE_ACTION_OCCUPY,
                    UPDATE_LOC, UPDATE_OCCUPANT)


class Occupant:
    """Class to define an occupant on a grid."""

    def __init__(self, occupant_id):
        """Set up occupant."""
        self.occupant_id = occupant_id

    def __repr__(self):
        """Display occupant."""
        return f'Occupant(id={self.occupant_id})'

    def __eq__(self, other):
        """Equality of occupants."""
        return self.occupant_id == other.occupant_id

    def get_id(self):
        """Get the ID of the occupant."""
        return self.occupant_id


class Occupants(set):
    """Class to define a list of occupants."""


class OccupancyGridLocation(NamedTuple):
    """Class to define a location in the occupancy grid."""

    # pylint: disable=too-few-public-methods
    # Bug in pylint
    x: np.intp
    y: np.intp
    z: np.intp


class OccupancyGridAction(Enum):
    """Class to define acctions for an occupancy grid update."""

    OCCUPY = UPDATE_ACTION_OCCUPY
    LEAVE = UPDATE_ACTION_LEAVE


class OccupancyGridUpdate():
    """Class to define an occupancy grid update."""

    def __init__(self, update_str=None, update_data=None):
        """Set up occupancy grid update."""
        if update_str is None and update_data is None:
            raise ValueError("Either update_str or update_data must be set")
        if update_str is not None and update_data is not None:
            raise ValueError("Only update_str or update_data may be set"
                             "but not both")
        if update_data is not None:
            self.update_data = update_data
            self.update_str = self.encode_str(update_data)
        else:
            self.update_str = update_str
            self.update_data = self.decode_str(update_str)

    def __repr__(self):
        """Display occupancy grid update."""
        return f'OccupancyGridUpdate(action={self.update_data.action}, ' \
               f'loc={self.update_data.loc})'

    def __str__(self):
        """Display string representation of occupancy grid update."""
        return self.update_str

    def __eq__(self, other):
        """Equality of updates."""
        return self.update_data == other.update_data

    @staticmethod
    def validate(update_data: dict, raise_error=False) -> bool:
        """
        Validate an update dict.

        Note: This is a very simple validation, it only verifies
        the correct combnations of top level keys.
        TODO: Schema-based validation with voluptuous
        """
        if UPDATE_ACTION not in update_data:
            if raise_error:
                raise ValueError('All updates must have an action')
            return False
        if UPDATE_LOC not in update_data:
            if raise_error:
                raise ValueError('All updates must have a location')
            return False
        if (update_data[UPDATE_ACTION] == UPDATE_ACTION_LEAVE or
                update_data[UPDATE_ACTION] == UPDATE_ACTION_OCCUPY):
            if UPDATE_OCCUPANT not in update_data:
                if raise_error:
                    raise ValueError('This action requires an occupant')
                return False
        return True

    @staticmethod
    def encode_str(update_data: dict) -> str:
        """Encode an update as a string."""
        OccupancyGridUpdate.validate(update_data, raise_error=True)
        return dumps(update_data)

    @staticmethod
    def decode_str(update_str: str) -> dict:
        """Dencode an update to a dict."""
        data = loads(update_str)
        OccupancyGridUpdate.validate(data, raise_error=True)
        if UPDATE_LOC in data:
            loc = data[UPDATE_LOC]
            data[UPDATE_LOC] = OccupancyGridLocation(x=loc.x, y=loc.y, z=loc.z)
        if UPDATE_OCCUPANT in data:
            data[UPDATE_OCCUPANT] = Occupant(data[UPDATE_OCCUPANT])
        return data


class OccupancyGrid:
    """Class to define a grid of occupants."""

    def __init__(self, grid_id, size: np.intp):
        """Set up occupancy grid."""
        self.grid_id = grid_id
        self.size = size
        self.grid = np.full((size, size, size), Occupants)

    def __repr__(self):
        """Display occupancy grid."""
        return f'OccupancyGrid(id={self.grid_id}, size={self.size})'

    def num_occupying(self, loc: OccupancyGridLocation) -> int:
        """Get the number of occupants in a space in the grid."""
        return len(self.grid[loc.x][loc.y][loc.z])

    def occupy(self, loc: OccupancyGridLocation,
               occupant: Occupant) -> str:
        """
        Occupy a space in the grid with an occupant.

        loc (OccupancyGridLocation): location to occupy
        occupant (Occupant): occupant to place in coordinates
        ret_update (bool): return JSON update message
        """
        (self.grid[loc.x][loc.y][loc.z]).add(occupant)
        return OccupancyGridUpdate(update_data={
            UPDATE_ACTION: OccupancyGridAction.OCCUPY,
            UPDATE_LOC: loc,
            UPDATE_OCCUPANT: occupant
        })

    def leave(self, loc: OccupancyGridLocation,
              occupant: Occupant) -> OccupancyGridUpdate:
        """
        Leave a space in the grid with an occupant.

        loc (OccupancyGridLocation): location to leave
        occupant (Occupant): occupant to remove from coordinates
        ret_update (bool): return JSON update message
        """
        # discard() will not error in case gets out-of-sync or joins late
        (self.grid[loc.x][loc.y][loc.z]).discard(occupant)
        return OccupancyGridUpdate(update_data={
            UPDATE_ACTION: OccupancyGridAction.LEAVE,
            UPDATE_LOC: loc,
            UPDATE_OCCUPANT: occupant
        })

    def action(self, update: OccupancyGridUpdate) -> OccupancyGridUpdate:
        """Run an action given an update."""
        if update[UPDATE_ACTION] == OccupancyGridAction.OCCUPY:
            return self.occupy(loc=update[UPDATE_LOC],
                               occupant=update[UPDATE_OCCUPANT])
        if update[UPDATE_ACTION] == OccupancyGridAction.LEAVE:
            return self.leave(loc=update[UPDATE_LOC],
                              occupant=update[UPDATE_OCCUPANT])
        raise ValueError("Unknown action")
