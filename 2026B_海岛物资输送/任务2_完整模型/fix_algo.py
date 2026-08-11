with open(r'C:\Users\ming\Desktop\数模备赛\任务2_完整模型\论文正文.tex', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace algorithm package
content = content.replace(r'\usepackage{algorithm,algorithmic}',
    r'\usepackage{algorithm}\usepackage{algpseudocode}')

# Replace the online CP algorithm
old_algo = r'''\begin{algorithm}[htbp]
\caption{在线CP滚动决策}
\label{alg:online}
\begin{algorithmic}[1]
\FOR{ = 1$ 到 {\max}$}
    \STATE 观测当日天气 $
    \STATE 从 $ 出发，假设剩余天数天气均为 $，执行CP搜索
    \STATE 获得最优方案，展开为逐日动作序列
    \STATE 执行当日动作，状态转移 {t+1} \leftarrow \mathcal{T}(S_t, a_t, w_t)$
    \IF{抵达 $} \STATE \textbf{break} \ENDIF
\ENDFOR
\end{algorithmic}
\end{algorithm}'''

new_algo = r'''\begin{algorithm}[htbp]
\caption{在线CP滚动决策}
\label{alg:online}
\begin{algorithmic}[1]
\For{ = 1 \to T_{\max}$}
    \State 观测当日天气 $
    \State 从 $ 出发，假设剩余天数天气均为 $，执行CP搜索
    \State 获得最优方案，展开为逐日动作序列
    \State 执行当日动作，{t+1} \leftarrow \mathcal{T}(S_t, a_t, w_t)$
    \If{抵达 $}
        \State \textbf{break}
    \EndIf
\EndFor
\end{algorithmic}
\end{algorithm}'''

content = content.replace(old_algo, new_algo)

# Also fix the EVCP algorithm if present
old_algo2 = r'''\begin{algorithm}[htbp]
\caption{EVCP在线滚动决策（问题三）}
\label{alg:evcp}
\begin{algorithmic}[1]
\STATE 初始化  = (B, 100, 150, 100, 750, 200, 0, -, 1)$
\FOR{ = 1$ 到 {\max}$}'''

if old_algo2 in content:
    new_algo2 = r'''\begin{algorithm}[htbp]
\caption{EVCP在线滚动决策（问题三）}
\label{alg:evcp}
\begin{algorithmic}[1]
\State 初始化  = (B, 100, 150, 100, 750, 200, 0, -, 1)$
\For{ = 1 \to T_{\max}$}'''
    content = content.replace(old_algo2, new_algo2)
    content = content.replace(r'\STATE 观测当日天气 $', r'\State 观测当日天气 $')
    content = content.replace(r'\STATE 根据 $ 的实际消耗更新资源余额', r'\State 根据 $ 的实际消耗更新资源余额')
    content = content.replace(r'\STATE 从当前状态出发', r'\State 从当前状态出发')
    content = content.replace(r'\STATE 获得最优方案', r'\State 获得最优方案')
    content = content.replace(r'\STATE 执行当日动作', r'\State 执行当日动作')
    content = content.replace(r'\IF{抵达 $} \STATE \textbf{break} \ENDIF', r'\If{抵达 $}\State \textbf{break}\EndIf')
    content = content.replace(r'\ENDFOR', r'\EndFor')
    content = content.replace(r'\RETURN 最终', r'\State \Return 最终')
    content = content.replace(r'\REQUIRE', r'\Require')
    content = content.replace(r'\ENSURE', r'\Ensure')

# Also fix the sim algorithm if present
content = content.replace(r'\REQUIRE 路径骨架', r'\Require 路径骨架')
content = content.replace(r'\ENSURE 可行性标志及最终', r'\Ensure 可行性标志及最终')
content = content.replace(r'\STATE 初始化', r'\State 初始化')
content = content.replace(r'\FOR{每段', r'\For{每段')
content = content.replace(r'\FOR{移动', r'\For{移动')
content = content.replace(r'\IF{到达补给点', r'\If{到达补给点')
content = content.replace(r'\IF{到达作业点', r'\If{到达作业点')
content = content.replace(r'\IF{\geq 0$', r'\If{\geq 0$')
content = content.replace(r'\ELSE', r'\Else')
content = content.replace(r'\ENDIF', r'\EndIf')
content = content.replace(r'\ENDFOR', r'\EndFor')
content = content.replace(r'\RETURN 可行则返回', r'\State \Return 可行则返回')
content = content.replace(r'\RETURN 不可行', r'\State \Return 不可行')

with open(r'C:\Users\ming\Desktop\数模备赛\任务2_完整模型\论文正文.tex', 'w', encoding='utf-8') as f:
    f.write(content)

print('Fixed. File size:', len(content), 'bytes')
