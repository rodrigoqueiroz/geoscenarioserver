import pickle
import os
from pandas import read_csv
import numpy as np
import re

np.set_printoptions(suppress=True)

try:
    import lanelet2
    use_lanelet2_lib = True
except ImportError:
    import warnings
    string = "Could not import lanelet2."
    warnings.warn(string)
    use_lanelet2_lib = False
if use_lanelet2_lib:
    from lanelet2.projection import UtmProjector
    # from av2.geometry.interpolate import compute_midpoint_line

class InteractionDataset():
    def __init__(self, config, train=True):
        self.config = config
        self.train = train

        if self.train:
            self.filename_pattern = re.compile(r'^(\w+)_train_\d+.csv$')
            # self.mapping_filename = 'mapping_train.pkl'
            ### MODIFICATION
            self.m2i_ig_labels_path = 'm2i_ig_labels_train'
            self.n_samples = 47584
            self.tracks_reformatted = self.config['tracks_train_reformatted']
            # self.graphs = self.config['graphs_train']
            # self.preprocess_path = self.config["preprocess_train"]
        else:
            self.filename_pattern = re.compile(r'^(\w+)_val_\d+.csv$')
            # self.mapping_filename = 'mapping_val.pkl'
            ### MODIFICATION
            self.m2i_ig_labels_path = 'm2i_ig_labels_val'
            self.n_samples = 11794
            self.tracks_reformatted = self.config['tracks_val_reformatted']
            # self.graphs = self.config['graphs_val']
            # self.preprocess_path = self.config["preprocess_val"]
        
        # load mapping dictionary
        # with open(os.path.join(self.config['dataset_path'], self.mapping_filename), "rb") as f:
        #     self.mapping = pickle.load(f)

        # if not os.path.isdir(self.graphs):
        #     os.makedirs(self.graphs)   

        # if self.config['preprocess']:
        #     if self.train:
        #         self.split = np.load(self.config['preprocess_train'], allow_pickle=True)
        #     else:
        #         self.split = np.load(self.config['preprocess_val'], allow_pickle=True)
        # else:
        #     if use_lanelet2_lib:
        #         self.projector = UtmProjector(lanelet2.io.Origin(0, 0))
        #         self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
        #                                             lanelet2.traffic_rules.Participants.Vehicle)  
    
    def __len__(self):
        return self.n_samples

    def read_interaction_data(self, csv_path):
        # csv_path = os.path.join(self.tracks_reformatted, csv_file)

        """TRACK_ID,FRAME_ID,TIMESTAMP_MS,AGENT_TYPE,X,Y,VX,VY,PSI_RAD,LENGTH,WIDTH"""
        city = 'DEU'
        df = read_csv(csv_path)

        agt_ts = np.sort(np.unique(df['timestamp_ms'].values))
        timestamp_mapping = dict()
        for i, ts in enumerate(agt_ts):
            timestamp_mapping[ts] = i

        trajs = np.concatenate((
            df.x.to_numpy().reshape(-1, 1),
            df.y.to_numpy().reshape(-1, 1)
        ), 1)

        vels = np.concatenate((
            df.vx.to_numpy().reshape(-1, 1),
            df.vy.to_numpy().reshape(-1, 1)
        ), 1)

        psirads = df.psi_rad.to_numpy().reshape(-1, 1)

        agenttypes = df.agent_type
        agenttypes = np.array([1 if x == 'car' else 0 for x in agenttypes]).reshape(-1, 1)

        shapes = np.concatenate((
            df.length.to_numpy().reshape(-1, 1),
            df.width.to_numpy().reshape(-1, 1)
        ), 1)

        steps = [timestamp_mapping[x] for x in df['timestamp_ms'].values]
        steps = np.asarray(steps, np.int64)

        # We don't group by agent_type as we predict futures of all agents in the scene
        objs = df.groupby(['track_id']).groups 
        keys = list(objs.keys())
        ctx_trajs, ctx_steps, ctx_vels, ctx_psirads, ctx_shapes, ctx_agenttypes = [], [], [], [], [], []
        for key in keys:
            idcs = objs[key]
            ctx_trajs.append(trajs[idcs])
            ctx_steps.append(steps[idcs])
            ctx_vels.append(vels[idcs])
            ctx_psirads.append(psirads[idcs])
            ctx_shapes.append(shapes[idcs])
            ctx_agenttypes.append(agenttypes[idcs])           

        data = dict()
        data['city'] = city 
        data['trajs'] = ctx_trajs
        data['steps'] = ctx_steps 
        data['vels'] = ctx_vels
        data['psirads'] = ctx_psirads
        data['shapes'] = ctx_shapes
        data['agenttypes'] = ctx_agenttypes

        return data

# For Testing
if __name__ == '__main__':
    config = {}
    config['dataset_version'] = '1.2'
    config['recording_name'] = 'DR_DEU_Merging_MT'
    config['dataset_dir'] = '/mnt/Data/Research_Dataset/INTERACTION-Dataset-DR-multi-v1_2/val'
    config['map_dir'] = '/mnt/Data/Research_Dataset/INTERACTION-Dataset-DR-multi-v1_2/maps/'
    config['tracks_train_reformatted'] = None
    config['tracks_val_reformatted'] = None
    config["preprocess_train"] = None

    dataset_obj = InteractionDataset(config=config, train=False)
    data = dataset_obj.read_interaction_data("/mnt/Data/Research_Dataset/INTERACTION-Dataset-DR-multi-v1_2/val/DR_DEU_Merging_MT_val.csv")
    print(data)