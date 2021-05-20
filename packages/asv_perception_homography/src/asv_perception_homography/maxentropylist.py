"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import threading
import numpy as np
from scipy.stats import wasserstein_distance

class MaxEntropyList(object):
    "List which fills to size n, then attempts to maximize the entropy of the list for subsequent additions"

    def __init__( self, n ):
        self.items = []
        self.n = n
        self.lock = threading.Lock()

    def is_full( self ):
        return len(self.items)==self.n

    def append( self, item, item_np_fn ):
        "Append an item to the list if it increases max entropy, or the list is not yet full"

        # TODO:  just pass in the np array

        self.items.append(item)

        if len(self.items) <= self.n:
            return

        # full list, decide what to purge
        means = np.array([ np.mean( item_np_fn( x ) ) for x in self.items ])
        vars = np.array([ np.var( item_np_fn( x ) ) for x in self.items ])

        # https://datascience.stackexchange.com/a/54385
        # compute the wasserstein distance of each element if it were removed from the set
        values = [] # sum of wasserstein distances for each item's attributes

        for i in range(len(self.items)):
            item=self.items[i]
            wmean = wasserstein_distance( np.delete( means, i), means )
            wvar = wasserstein_distance( np.delete( vars, i ), vars )
            values.append(wmean+wvar)

        index_min = min(range(len(values)), key=values.__getitem__)
        self.items.pop( index_min )