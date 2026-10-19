#!/usr/bin/env python3
"""Run the production C++ estimators offline; summarize without closed-loop claims.
Run after sourcing devel/setup.bash. All generated files belong outside the repo.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
import pandas as pd
from scipy import signal
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATES = ['20261017_152939', '20261019_144137', '20261019_162704']
METHODS = ['window_opt'] + [f'ndob_{r}_{c}' for r in (5, 10, 20, 40) for c in (3, 5, 10, 20)]
SHOW = ['window_opt', 'ndob_20_5', 'ndob_20_20']


def vec(d, prefix):
    return d[[prefix + a for a in 'xyz']].to_numpy(float)


def rms(v):
    return float(np.sqrt(np.mean(np.square(v)))) if np.size(v) else np.nan


def nearest(t, query):
    i = np.clip(np.searchsorted(t, query), 1, len(t)-1)
    return np.where(abs(t[i]-query) < abs(t[i-1]-query), i, i-1)


def runs(mask):
    edges = np.diff(np.r_[False, np.asarray(mask, bool), False].astype(int))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def summary(v, dt=.01):
    if len(v) < 2:
        return {}
    return dict(mean_x=float(v[:, 0].mean()), mean_y=float(v[:, 1].mean()),
                mean_z=float(v[:, 2].mean()), fluct_rms_N=rms(np.linalg.norm(v-v.mean(axis=0), axis=1)),
                p95_norm_N=float(np.percentile(np.linalg.norm(v, axis=1), 95)),
                rate_rms_N_s=rms(np.linalg.norm(np.diff(v, axis=0)/dt, axis=1)))


def synthetic_report(root):
    d = pd.read_csv(root/'synthetic.csv')
    rows = []
    for (scenario, axis), group in d.groupby(['scenario', 'axis']):
        t = group.t.to_numpy(); truth = group['truth_'+'xyz'[axis]].to_numpy()
        for m in METHODS:
            y = group[m+'_f'+'xyz'[axis]].to_numpy()
            row = dict(scenario=scenario, axis=axis, method=m, rmse_N=rms(y-truth),
                       steady_error_N=float(np.mean((y-truth)[t>=8])),
                       compute_median_us=float(group[m+'_us'].median()),
                       compute_p95_us=float(group[m+'_us'].quantile(.95)),
                       compute_p99_us=float(group[m+'_us'].quantile(.99)))
            if scenario == 'step':
                times=[]
                for threshold in (.1,.9):
                    k=np.flatnonzero((t>=2)&(y>=threshold))
                    times.append(t[k[0]] if len(k) else np.nan)
                row.update(rise_10_90_s=times[1]-times[0], onset_to_90_s=times[1]-2,
                           overshoot_pct=100*(y[t>=2].max()-1))
            if scenario.startswith('sine_'):
                freq=float(scenario[5:]); use=t>=2
                basis=np.column_stack([np.sin(2*np.pi*freq*t[use]),np.cos(2*np.pi*freq*t[use]),np.ones(use.sum())])
                coeff=np.linalg.lstsq(basis,y[use],rcond=None)[0]
                row.update(amplitude_ratio=float(np.hypot(*coeff[:2])/.5),
                           phase_deg=float(np.degrees(np.arctan2(coeff[1],coeff[0]))))
            if scenario == 'outlier': row['peak_false_force_N']=float(abs(y).max())
            if scenario == 'pulse': row['pulse_rmse_N']=rms((y-truth)[(t>=2)&(t<3)])
            rows.append(row)
    metrics=pd.DataFrame(rows);metrics.to_csv(root/'synthetic_metrics.csv',index=False)
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for ax,scenario in zip(axes.flat,['step','pulse','outlier','step_noise']):
        g=d[(d.scenario==scenario)&(d.axis==0)]
        ax.plot(g.t,g.truth_x,'k--',label='truth')
        for m in SHOW: ax.plot(g.t,g[m+'_fx'],label=m)
        ax.set(xlim=(1.8,3),title=scenario,xlabel='time (s)',ylabel='force (N)');ax.grid();ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(root/'synthetic_response.png',dpi=150);plt.close(fig)
    return metrics


def relative_positions(bag, query, output):
    """Only use explicit world -> camera_init and camera_init -> body TF chains.
    Interpolate positions at source stamps with <= 0.25 s bracketing gaps.
    This is a recorded coordinate convention, not external calibration evidence.
    """
    import rosbag
    static={}; data={0:[],1:[]}; latencies={0:[],1:[]}
    with rosbag.Bag(str(bag)) as b:
        for topic,msg,arrival in b.read_messages(topics=['/tf_static','/tf']):
            for tr in msg.transforms:
                parent=tr.header.frame_id.lstrip('/');child=tr.child_frame_id.lstrip('/')
                v=tr.transform.translation;q=tr.transform.rotation
                for u in data:
                    if parent=='world' and child==f'UAV{u}/camera_init':
                        static[u]=(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(), np.array([v.x,v.y,v.z]))
                    if parent==f'UAV{u}/camera_init' and child==f'UAV{u}/body':
                        data[u].append([tr.header.stamp.to_sec(),v.x,v.y,v.z])
                        latencies[u].append(arrival.to_sec()-tr.header.stamp.to_sec())
    valid=np.ones(len(query),bool);positions={}
    for u in data:
        if u not in static or len(data[u])<2:
            return np.full(len(query),np.nan), {'available':False,'reason':'missing explicit TF chain'}
        a=np.array(data[u]);a=a[np.argsort(a[:,0])];a=a[np.r_[True,np.diff(a[:,0])>0]]
        rotation,translation=static[u];xyz=a[:,1:]@rotation.T+translation
        positions[u]=np.column_stack([np.interp(query,a[:,0],xyz[:,j]) for j in range(3)])
        i=np.clip(np.searchsorted(a[:,0],query),1,len(a)-1)
        valid &= (query>=a[0,0])&(query<=a[-1,0])&((a[i,0]-a[i-1,0])<=.25)
    latency_p95={str(u):float(np.percentile(np.abs(latencies[u]),95)) for u in data}
    clock_consistent=all(v<=.25 for v in latency_p95.values())
    valid &= clock_consistent
    delta=positions[1]-positions[0];delta[~valid]=np.nan
    distance=np.linalg.norm(delta,axis=1)
    pd.DataFrame(dict(t=query,dx=delta[:,0],dy=delta[:,1],dz=delta[:,2],distance=distance,valid=valid)).to_csv(output,index=False)
    return distance,dict(available=clock_consistent,source_to_receipt_abs_p95_s=latency_p95,valid_fraction=float(valid.mean()),max_interpolation_gap_s=.25,
                         source='recorded /tf_static and /tf source stamps; no external calibration validation',
                         world_translations={str(u):static[u][1].tolist() for u in static})


def real_report(root,date,bag):
    d=pd.read_csv(root/(date+'.csv')); rec=pd.read_csv(root/(date+'_recorded.csv'))
    t=d.t.to_numpy();active=d.window_opt_active.to_numpy(bool); rel=t-t[0]
    distance,tf_info=relative_positions(bag,t,root/(date+'_relative.csv'))
    (root/(date+'_relative_metadata.json')).write_text(json.dumps(tf_info,indent=2))
    baseline=[]
    for kind,field in [('estimated','f'),('applied','a')]:
        r=rec[rec.kind==kind]; rt=r.t.to_numpy();i=nearest(t,rt)
        measured=vec(r,'');replayed=vec(d,'window_opt_'+field)[i]
        for scope,mask in [('all',np.ones(len(i),bool)),('reconstructed_active',active[i])]:
            delta=replayed[mask]-measured[mask]
            baseline.append(dict(bag=date,kind=kind,scope=scope,count=int(mask.sum()),
                                 vector_rmse_N=rms(np.linalg.norm(delta,axis=1)),
                                 bias_x=float(np.mean(delta[:,0])) if len(delta) else np.nan,
                                 bias_y=float(np.mean(delta[:,1])) if len(delta) else np.nan,
                                 bias_z=float(np.mean(delta[:,2])) if len(delta) else np.nan,
                                 p95_time_match_ms=float(np.percentile(abs(t[i]-rt)*1000,95))))
    pd.DataFrame(baseline).to_csv(root/(date+'_baseline_validation.csv'),index=False)
    # Segment by recorded mode and compensation activation; split at distance bins
    # in a separate table, never interpret proximity as force ground truth.
    labels=d['mode'].astype(str)+'/'+d.trajectory.astype(str)+'/'+d.window_opt_active.astype(str)+'/precision'+d.precision.astype(str)+'/recovery'+d.recovery.astype(str)+'/direct_land'+d.direct_land.astype(str)
    blocks=np.r_[0,np.flatnonzero(labels.to_numpy()[1:]!=labels.to_numpy()[:-1])+1,len(d)]
    rows=[]; segments=[]
    for start,end in zip(blocks[:-1],blocks[1:]):
        g=d.iloc[start:end];n=end-start
        if n<2:continue
        err=vec(g,'pos_')-vec(g,'ref_');finite=np.isfinite(err).all(axis=1)
        segments.append(dict(start_s=rel[start],end_s=rel[end-1],duration_s=n*.01,label=labels.iloc[start],
                             position_rms_m=rms(np.linalg.norm(err[finite],axis=1)),
                             position_p95_m=float(np.percentile(np.linalg.norm(err[finite],axis=1),95)) if finite.any() else np.nan,
                             angular_rate_rms_rad_s=rms(np.linalg.norm(vec(g,'omega_'),axis=1)),
                             median_distance_m=float(np.nanmedian(distance[start:end])) if np.isfinite(distance[start:end]).any() else np.nan))
        if not active[start]:continue
        for m in METHODS:
            for field in ('f','a'):
                v=vec(g,m+'_'+field)
                rows.append(dict(bag=date,start_s=rel[start],duration_s=n*.01,mode=g['mode'].iloc[0],method=m,field=field,
                                 compute_p50_us=g[m+'_us'].median(),compute_p95_us=g[m+'_us'].quantile(.95),compute_p99_us=g[m+'_us'].quantile(.99),**summary(v)))
    pd.DataFrame(segments).to_csv(root/(date+'_segments.csv'),index=False)
    pd.DataFrame(rows).to_csv(root/(date+'_segment_metrics.csv'),index=False)
    whole=[]
    for m in METHODS:
        for field in ('f','a'):
            # RMS rate only adjacent active ticks; no artificial derivatives across gaps.
            v=vec(d,m+'_'+field);use=active[1:]&active[:-1]
            record=dict(bag=date,method=m,field=field,active_seconds=float(active.sum()*.01),**summary(v[active]),
                        compute_p50_us=d.loc[active,m+'_us'].median(),compute_p95_us=d.loc[active,m+'_us'].quantile(.95),compute_p99_us=d.loc[active,m+'_us'].quantile(.99))
            record['rate_rms_N_s']=rms(np.linalg.norm(np.diff(v,axis=0)[use]/.01,axis=1))
            whole.append(record)
    pd.DataFrame(whole).to_csv(root/(date+'_metrics.csv'),index=False)
    proximity=[]
    for low,high in [(0,1),(1,2),(2,4),(4,np.inf)]:
        mask=active&np.isfinite(distance)&(distance>=low)&(distance<high)
        if mask.sum()<2:continue
        for m in METHODS:
            v=vec(d,m+'_f');stats=summary(v[mask])
            stats['rate_rms_N_s']=rms(np.linalg.norm(np.diff(v,axis=0)[mask[1:]&mask[:-1]]/.01,axis=1))
            proximity.append(dict(distance_min=low,distance_max=high,count=int(mask.sum()),method=m,**stats))
    pd.DataFrame(proximity).to_csv(root/(date+'_proximity_metrics.csv'),index=False)
    fig,axes=plt.subplots(4,1,figsize=(15,11),sharex=True)
    for j,ax in enumerate(axes[:3]):
        axis='xyz'[j];ax.plot(rel,d['raw_'+axis],color='.75',lw=.5,label='residual')
        for m in SHOW:ax.plot(rel,d[m+'_f'+axis],lw=.7,label=m)
        r=rec[rec.kind=='estimated'];ax.plot(r.t-t[0],r[axis],'k',lw=.5,alpha=.65,label='recorded window')
        ax.set(ylabel=f'F{axis} (N)');ax.grid();ax.legend(ncol=5,fontsize=8)
    for m in SHOW:axes[3].plot(rel,np.linalg.norm(vec(d,m+'_a'),axis=1),label=m)
    axes[3].set(ylabel='applied norm (N)',xlabel='time since recording (s)');axes[3].grid();axes[3].legend()
    fig.tight_layout();fig.savefig(root/(date+'_forces.png'),dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(15,8),sharex=True)
    r=rec[rec.kind=='applied']
    for j,ax in enumerate(axes):
        axis='xyz'[j]
        for m in SHOW:ax.plot(rel,d[m+'_a'+axis],lw=.7,label=m)
        ax.plot(r.t-t[0],r[axis],'k',lw=.5,alpha=.65,label='recorded applied')
        ax.set(ylabel=f'applied F{axis} (N)');ax.grid();ax.legend(ncol=4,fontsize=8)
    axes[-1].set(xlabel='time since recording (s)')
    fig.tight_layout();fig.savefig(root/(date+'_applied.png'),dpi=150);plt.close(fig)
    # PSD of the longest uninterrupted compensation interval, no concatenation.
    spans=runs(active); spectral=[]
    if spans:
        start,end=max(spans,key=lambda p:p[1]-p[0]);fig,axes=plt.subplots(1,2,figsize=(12,4))
        for field,ax in zip(('f','a'),axes):
            for m in METHODS:
                v=vec(d,m+'_'+field)[start:end]
                freq,psd=signal.welch(v,fs=100,axis=0,nperseg=min(1024,len(v)))
                total=psd.sum(axis=1)
                for fr,power in zip(freq,total):spectral.append(dict(method=m,field=field,frequency_Hz=fr,psd_N2_Hz=power,start_s=rel[start],end_s=rel[end-1]))
                if m in SHOW:ax.semilogy(freq,total,label=m)
            ax.set(xlim=(0,25),xlabel='frequency (Hz)',ylabel='PSD (N^2/Hz)',title='estimated' if field=='f' else 'applied');ax.grid();ax.legend()
        fig.tight_layout();fig.savefig(root/(date+'_spectra.png'),dpi=150);plt.close(fig)
    pd.DataFrame(spectral).to_csv(root/(date+'_spectra.csv'),index=False)
    r=rec[rec.kind=='estimated'];intervals=np.diff(r.t)
    timing=dict(bag=date,recorded_publication_dt_p50_ms=float(np.median(intervals)*1000),
                recorded_publication_dt_p95_ms=float(np.percentile(intervals,95)*1000),
                recorded_publication_dt_p99_ms=float(np.percentile(intervals,99)*1000),
                note='old bags have receipt timestamps, not actual calculation timestamps; scheduling and transport mixed',
                input_accepted=int(d.new_sample.sum()),held_control_fraction=float((~d.new_sample.astype(bool)&d.input_valid.astype(bool)).mean()))
    (root/(date+'_timing.json')).write_text(json.dumps(timing,indent=2))
    return pd.DataFrame(whole),pd.DataFrame(baseline)


def markdown_table(frame, cols):
    lines=['| '+' | '.join(cols)+' |','| '+' | '.join(['---']*len(cols))+' |']
    for _,row in frame.iterrows():
        lines.append('| '+' | '.join(f'{row[c]:.4g}' if isinstance(row[c],(float,np.floating)) else str(row[c]) for c in cols)+' |')
    return '\n'.join(lines)


def report(root,synth,real,baseline):
    step=synth[(synth.scenario=='step')&(synth.axis==0)].set_index('method')
    pulse=synth[(synth.scenario=='pulse')&(synth.axis==0)].set_index('method')
    outlier=synth[(synth.scenario=='outlier')&(synth.axis==0)].set_index('method')
    grid=step[['rise_10_90_s','onset_to_90_s','overshoot_pct','compute_p95_us']].copy()
    grid['outlier_peak_N']=outlier.peak_false_force_N
    grid['pulse_rmse_N']=pulse.pulse_rmse_N
    for date in DATES:
        g=real[(real.bag==date)&(real.field=='f')].set_index('method')
        grid[date+'_fluct_N']=g.fluct_rms_N
        a=real[(real.bag==date)&(real.field=='a')].set_index('method')
        grid[date+'_applied_rate_N_s']=a.rate_rms_N_s
    grid=grid.reindex(METHODS).reset_index();grid.to_csv(root/'parameter_grid.csv',index=False)
    fig,ax=plt.subplots(figsize=(10,6))
    ax.scatter(grid.rise_10_90_s,grid[DATES[0]+'_fluct_N'])
    for _,r in grid.iterrows():ax.annotate(r.method,(r.rise_10_90_s,r[DATES[0]+'_fluct_N']),fontsize=7)
    ax.set(xlabel='synthetic step 10-90% rise (s)',ylabel='Oct17 real force fluctuation RMS (N)',title='Full parameter grid: response vs recorded-input fluctuation');ax.grid()
    fig.tight_layout();fig.savefig(root/'parameter_tradeoff.png',dpi=150);plt.close(fig)
    text='''# AeroThrow NDOB 与当前窗口 Huber 估计器：离线比较\n\n此报告由生产 C++ 估计器生成。默认在线方法仍为 window_opt。比较限定为当前窗口参数（20 样本、var_weight=80、3 Hz 输出低通）及快照参数，不能推断换用 NDOB 后的位置误差或飞行稳定性。\n\n## 先验证原方法回放\n\n历史数据缺少完整运行参数、观察器/控制器定时器相位和精确回调顺序。本工具按录包接收时间、源时间戳、100 Hz 标称控制/观察器事件及 UAV1 ROS 日志重建；传输延迟、丢包、模式日志延迟、恢复流程细节会形成偏差。未调整质量、kf、窗口或截止频率来拟合历史曲线。以下偏差用于衡量重建质量，不是算法精度。\n\n'''
    text+=markdown_table(baseline[baseline.scope=='reconstructed_active'],['bag','kind','count','vector_rmse_N','bias_x','bias_y','bias_z'])
    text+='\n\n历史波形不满足逐点一致，因此以下实测比较仅表示“同一重建输入下的算法差异”，不能宣称精确复现历史控制执行。所有基线误差（含全时段）、时间匹配偏差见 *_baseline_validation.csv。\n\n## 完整参数扫描\n\n阶跃为充分准备后的 1 N 已知输入，每轴分别施加；10%—90% 上升时间不包含 20 个有效样本的补偿接入等待，也不包含输入预滤波。当前回放包含输入预滤波、同步、门控、分轴增益及合力限幅；已取消补偿端额外 Z 轴低通和变化率限制。旧录包中这两项曾启用，当前回放不重现旧补偿链路。\n\n'
    text+=markdown_table(grid,['method','rise_10_90_s','onset_to_90_s','outlier_peak_N','pulse_rmse_N','compute_p95_us',DATES[0]+'_fluct_N'])
    text+='\n\n![响应与波动](parameter_tradeoff.png)\n\n![合成响应](synthetic_response.png)\n\n合成数据含三轴零外力、倾斜姿态、阶跃、0.2 s 脉冲、0.5/1/2/5 Hz 正弦、孤立 3 N 异常、0.4 N 固定偏置及 σ=0.12 N 高斯噪声。完整稳态误差、过冲、幅相、异常影响、RMS、计算 P50/P95/P99 在 synthetic_metrics.csv。固定偏置对两者均无法与真实外力区分。\n\n## 各录包独立复核\n\n'
    for date in DATES:
        text+=f'### {date}\n\n'+markdown_table(real[(real.bag==date)&real.method.isin(SHOW)],['method','field','active_seconds','fluct_rms_N','rate_rms_N_s','compute_p50_us','compute_p95_us','compute_p99_us'])
        text+=f'\n\n![{date} 外力与补偿]({date}_forces.png)\n\n![{date} 实际补偿]({date}_applied.png)\n\n![{date} 频谱]({date}_spectra.png)\n\n'
    text+='''## 解释边界与后续实飞\n\n真实数据中高频分量称为波动，不能全部视作传感器噪声。*_segments.csv 按模式、轨迹执行/悬停、补偿启停、精降和恢复标签分段，位置误差和原始 IMU 角速度只描述已经发生的飞行；参考路径首点采用最近记录值，非每个真实 MPC 周期的精确参考。*_events.tsv 保留 UAV1 的门控/恢复日志，不能用另一架机的日志代替。\n\n两机相对位置只在录包中存在完整 world→camera_init→body 链、源时间戳能插值、相邻间隔不超过 0.25 s 且两机源时间到录包接收时间绝对偏差 P95 均不超过 0.25 s 时统计；TF 的 world 标定未获独立真值验证。距离分组只作环境标签，不是尾流真值。跨日期的飞行参数、环境、飞行轨迹不同，不能横向当成算法效果。\n\n历史 force 话题无 header，*_timing.json 的间隔是录包接收间隔，混合调度及传输抖动；新版本 force_timing 记录实际计算调用间隔（秒）、耗时（微秒）、有效样本数。NDOB 每次实际调用固定推进 0.01 s，恢复时跳过、不补算。这里的本机离线耗时不等于机载实时最坏耗时。\n\n保留两个实飞候选：20 s⁻¹ / 5 Hz（减轻输出高频与异常影响）、20 s⁻¹ / 20 Hz（较快脉冲跟随，用于检验响应收益）。这是依据 10 月 17 日完整扫描作出的实验取舍，不代表数学最优；两组 10 月 19 日数据均显示响应收益伴随更大的估计波动及补偿变化率。不能以计算更快推出跟踪更好。尚无配对实飞，不能判定 NDOB 更适用于通道双机飞行。后续固定路线、速度、控制参数及前机运动，交错进行每种方法至少三轮；记录电压及相对位置，比较位置误差 RMS/P95、去均值摆动、实际角速度、指令变化和实际补偿。起飞阶段不启用外力补偿，本次结果不能证明改善起飞抖动。\n'''
    (root/'report.md').write_text(text)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--bag-root',type=Path,default=Path.home()/'.ros/autotrans_mpc_logs')
    parser.add_argument('--replay',default='force_estimator_replay')
    parser.add_argument('--config',required=True,type=Path);parser.add_argument('--model',required=True,type=Path)
    parser.add_argument('--reuse',action='store_true',help='Reuse existing C++ output explicitly (no automatic cache validation)')
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    snap=root/'snapshot';snap.mkdir(exist_ok=True)
    for src,name in [(args.config,'mpc.yaml'),(args.model,'model.yaml')]:
        dst=snap/name
        if not args.reuse or not dst.exists():
            if src.resolve()!=dst.resolve():shutil.copyfile(src,dst)
    manifest={name:hashlib.sha256((snap/name).read_bytes()).hexdigest() for name in ['mpc.yaml','model.yaml']}
    manifest['provenance']='current workspace snapshot; historical complete effective parameters unavailable'
    (snap/'provenance.json').write_text(json.dumps(manifest,indent=2))
    base=[args.replay,'--config',str(snap/'mpc.yaml'),'--model',str(snap/'model.yaml')]
    if not args.reuse or not (root/'synthetic.csv').exists():subprocess.run(base+['--synthetic','true','--output',str(root/'synthetic.csv')],check=True)
    bags={}
    for date in DATES:
        found=sorted(args.bag_root.glob(date+'*/*.bag'))
        if len(found)!=1:raise RuntimeError(f'{date}: expected exactly one bag, found {found}')
        bags[date]=found[0]
        if not args.reuse or not (root/(date+'.csv')).exists():subprocess.run(base+['--bag',str(found[0]),'--output',str(root/date)],check=True)
    synth=synthetic_report(root);real=[];baseline=[]
    for date,bag in bags.items():
        a,b=real_report(root,date,bag);real.append(a);baseline.append(b)
    report(root,synth,pd.concat(real,ignore_index=True),pd.concat(baseline,ignore_index=True))
    print(root/'report.md')


if __name__=='__main__':main()
