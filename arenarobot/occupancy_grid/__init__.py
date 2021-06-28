"""
__init__.py: Definitions for the occupancy grid.

Created by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import numpy as np


class Occupant:
    """Class to define an occupant on a grid."""

    def __init__(self, occupant_id):
        """Set up occupant."""
        self.occupant_id = occupant_id

    def __repr__(self):
        """Display occupant."""
        return f'Occupant(id={self.occupant_id})'

    def get_id(self):
        """Get the ID of the occupant."""
        return self.occupant_id


class Occupants(set):
    """Class to define a list of occupants."""


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

    def num_occupying(self, grid_x: np.intp, grid_y: np.intp,
                      grid_z: np.intp) -> int:
        """Get the number of occupants in a space in the grid."""
        return len(self.grid[grid_x][grid_y][grid_z])

    def occupy(self, grid_x: np.intp, grid_y: np.intp, grid_z: np.intp,
               occupant: Occupant) -> None:
        """Occupy a space in the grid with an occupant."""
        (self.grid[grid_x][grid_y][grid_z]).add(occupant)

    def remove(self, grid_x: np.intp, grid_y: np.intp, grid_z: np.intp,
               occupant: Occupant) -> None:
        """Vacate a space in the grid with an occupant."""
        # discard() will not error in case gets out-of-sync or joins late
        (self.grid[grid_x][grid_y][grid_z]).discard(occupant)
