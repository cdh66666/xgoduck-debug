"""PPO config only; no simulation or training on import."""
STD_RESET=.02
LEARNING_RATE=3e-5
ENTROPY=.0005


def agent_config():
    import json
    from pathlib import Path
    frozen=json.loads((Path(__file__).resolve().parent/'frozen-manifest.json').read_text(encoding='utf-8'))
    parent=frozen['cpu_training_parent']['checkpoint']
    import hashlib
    return dict(algorithm=dict(class_name='rsl_rl.algorithms:PPO', value_loss_coef=1.,
        use_clipped_value_loss=True, clip_param=.2, entropy_coef=ENTROPY,
        num_learning_epochs=5, num_mini_batches=4, learning_rate=LEARNING_RATE,
        schedule='fixed', gamma=.99, lam=.95, desired_kl=.01, max_grad_norm=1.,
        rnd_cfg=None, symmetry_cfg=None),
        actor=dict(class_name='residual_model:ResidualActor', hidden_dims=[512,256,128], activation='elu',
                   parent_checkpoint=parent,parent_checkpoint_sha256=hashlib.sha256(Path(parent).read_bytes()).hexdigest(),
                   residual_scale=.2,
                   obs_normalization=True, distribution_cfg=dict(class_name='GaussianDistribution', init_std=STD_RESET, std_type='scalar')),
        critic=dict(class_name='rsl_rl.models:MLPModel', hidden_dims=[512,256,128], activation='elu', obs_normalization=True),
        obs_groups={'actor':['actor'], 'critic':['critic']}, num_steps_per_env=24,
        save_interval=100, logger='offline_native', check_for_nan=True)
