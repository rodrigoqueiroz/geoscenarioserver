Behavior trees organized by agent type.

Given the parameter `--btree-locations=<L>` (`btrees` by default), the btree file is located as follows.

For a vehicle defined as
```
<tag k='btree' v='<B>.btree' />
<tag k='btype' v='SDV' />
<tag k='gs' v='vehicle' />
```
the path is `<L>/sdv/<B>.btree`.

For a pedestrian defined as
```
<tag k='btree' v='<B>.btree' />
<tag k='btype' v='SP' />
<tag k='gs' v='pedestrian' />
```
the path is `<L>/sp/<B>.btree`.
