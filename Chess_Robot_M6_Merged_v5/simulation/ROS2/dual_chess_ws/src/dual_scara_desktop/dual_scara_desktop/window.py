"""Qt input/display only. No ROS clients, processes or match progression in widgets."""
from PySide6.QtCore import Qt,Signal,QRectF
from PySide6.QtGui import QColor,QPainter,QFont
from PySide6.QtWidgets import (QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QGridLayout,QComboBox,QPushButton,
                               QLabel,QLineEdit,QCheckBox,QGroupBox,QSplitter,QPlainTextEdit,QListWidget,
                               QTabWidget,QFileDialog,QDoubleSpinBox,QSlider,QScrollArea,QSizePolicy)
from .modes import MODES,SCENARIOS

SYMBOLS=dict(zip('KQRBNPkqrbnp','♔♕♖♗♘♙♚♛♜♝♞♟'))

class ChessBoard(QWidget):
    square_clicked=Signal(str)
    def __init__(self):
        super().__init__();self.setMinimumSize(300,300);self.fen='';self.selected=None;self.flipped=False;self.interactive=False;self.last_move=''
        self.setObjectName('chess_board')
    def set_position(self,fen,selected=None,flipped=False,last_move=''):
        self.fen,self.selected,self.flipped,self.last_move=fen,selected,flipped,last_move;self.update()
    def board_rect(self):
        side=max(8,min(self.width()-42,self.height()-42));return QRectF((self.width()-side)/2,(self.height()-side)/2,side,side)
    def square_centre(self,square):
        f=ord(square[0])-97;r=int(square[1])-1
        col,row=(7-f,r) if self.flipped else (f,7-r)
        rect=self.board_rect();s=rect.width()/8
        return rect.left()+(col+.5)*s,rect.top()+(row+.5)*s
    def mousePressEvent(self,event):
        if not self.interactive:return
        p=event.position();rect=self.board_rect()
        if not rect.contains(p):return
        col=min(7,int((p.x()-rect.left())/(rect.width()/8)));row=min(7,int((p.y()-rect.top())/(rect.height()/8)))
        f,r=(7-col,row) if self.flipped else (col,7-row)
        self.square_clicked.emit(chr(97+f)+str(r+1))
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing);rect=self.board_rect();size=rect.width()/8
        pieces={}
        try:
            for row,text in enumerate(self.fen.split()[0].split('/')):
                col=0
                for value in text:
                    if value.isdigit():col+=int(value)
                    else:pieces[chr(97+col)+str(8-row)]=value;col+=1
        except (IndexError,ValueError):pass
        p.setFont(QFont('DejaVu Sans',10))
        for row in range(8):
            for col in range(8):
                f,r=(7-col,row) if self.flipped else (col,7-row);square=chr(97+f)+str(r+1)
                box=QRectF(rect.left()+col*size,rect.top()+row*size,size,size)
                colour='#e5e9e5' if (f+r)%2 else '#4c736c'
                if square==self.selected:colour='#e6b75a'
                elif self.last_move and square in [self.last_move[:2],self.last_move[2:4]]:colour='#86b9a3'
                p.fillRect(box,QColor(colour))
                if square in pieces:
                    font=QFont('DejaVu Sans');font.setPixelSize(int(size*.77));p.setFont(font)
                    symbol=pieces[square]
                    p.setPen(QColor('#101c27'))
                    p.drawText(box,Qt.AlignCenter,SYMBOLS.get(symbol,symbol))
        p.setFont(QFont('DejaVu Sans',10));p.setPen(QColor('#bac8d3'))
        for col in range(8):
            file=chr(97+(7-col if self.flipped else col));p.drawText(QRectF(rect.left()+col*size,rect.bottom()+2,size,20),Qt.AlignCenter,file)
        for row in range(8):
            rank=str(row+1 if self.flipped else 8-row);p.drawText(QRectF(rect.left()-22,rect.top()+row*size,20,size),Qt.AlignCenter,rank)


class MainWindow(QMainWindow):
    start_requested=Signal();stop_requested=Signal();close_requested=Signal();move_requested=Signal(str)
    square_requested=Signal(str);joint_requested=Signal(str,float);apply_joints_requested=Signal()
    stop_motion_requested=Signal();restart_requested=Signal();failure_requested=Signal();vision_hold_requested=Signal(bool)
    promotion_requested=Signal();sync_requested=Signal();state_requested=Signal()
    def __init__(self,geometry,domain):
        super().__init__();self.allow_close=False;self.setWindowTitle('Dual SCARA • Local robot console');self.resize(1260,840)
        root=QWidget();self.setCentralWidget(root);layout=QVBoxLayout(root)
        heading=QLabel('DUAL SCARA  /  CHESS ROBOT');heading.setObjectName('heading');layout.addWidget(heading)
        self.status=QLabel('Stopped • select a mode and press Start');self.status.setWordWrap(True);self.status.setObjectName('status');layout.addWidget(self.status)
        split=QSplitter();layout.addWidget(split,1)
        left=QWidget();ll=QVBoxLayout(left);left.setMaximumWidth(360)
        self.mode=QComboBox();self.mode.setObjectName('mode_selector')
        for m in MODES:self.mode.addItem(m.title,m.key)
        self.mode.setMinimumContentsLength(20);self.mode.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        ll.addWidget(QLabel(f'LOCAL DESKTOP  •  ROS domain {domain}'));ll.addWidget(self.mode)
        self.description=QLabel(MODES[0].explanation);self.description.setWordWrap(True);ll.addWidget(self.description)
        ll.addWidget(QLabel('Geometry YAML (physical profile)'))
        row=QHBoxLayout();self.profile=QLineEdit(str(geometry));self.profile.setObjectName('geometry_file');self.profile.setToolTip(str(geometry));self.profile.textChanged.connect(self.profile.setToolTip);self.browse=QPushButton('Browse…');row.addWidget(self.profile,1);row.addWidget(self.browse);ll.addLayout(row)
        self.browse.clicked.connect(self.pick_profile)
        self.rviz=QCheckBox('Open RViz in a separate window');self.rviz.setChecked(True);ll.addWidget(self.rviz)
        ll.addWidget(QLabel('Human color / arm assignment'))
        self.human=QComboBox();self.human.addItem('White — robot1','white');self.human.addItem('Black — robot2','black');ll.addWidget(self.human)
        self.scenario=QComboBox();self.scenario.addItems(list(SCENARIOS));ll.addWidget(QLabel('Mock test scenario (resets simulated board)'));ll.addWidget(self.scenario)
        self.engine_path=QLineEdit('/usr/games/stockfish');ll.addWidget(QLabel('Stockfish executable'));ll.addWidget(self.engine_path)
        row=QHBoxLayout();self.start=QPushButton('Start mode');self.start.setObjectName('start_mode');self.stop=QPushButton('Stop mode');self.stop.setEnabled(False);row.addWidget(self.start);row.addWidget(self.stop);ll.addLayout(row)
        self.start.clicked.connect(self.start_requested);self.stop.clicked.connect(self.stop_requested)
        self.backend=QLabel('Backend: stopped');self.backend.setWordWrap(True);ll.addWidget(self.backend)
        self.pending=QLabel('No pending move');self.pending.setWordWrap(True);ll.addWidget(self.pending)
        self.safety=QLabel('Simulation only • geometry and mounting remain provisional.');self.safety.setWordWrap(True);ll.addWidget(self.safety)
        self.effective=QPlainTextEdit();self.effective.setReadOnly(True);self.effective.setMaximumHeight(160);ll.addWidget(QLabel('Effective ROS geometry'));ll.addWidget(self.effective)
        ll.addStretch(1)
        for label in left.findChildren(QLabel):label.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Minimum)
        self.motion_stop=QPushButton('■ Software motion stop');self.motion_stop.setObjectName('motion_stop');self.motion_stop.clicked.connect(self.stop_motion_requested)
        footer=QHBoxLayout();note=QLabel('Local simulation console • Software motion stop does not shut down the mode or replace hardware safety.');note.setWordWrap(True)
        footer.addWidget(note,1);footer.addWidget(self.motion_stop);layout.addLayout(footer)
        sidebar=QScrollArea();sidebar.setWidgetResizable(True);sidebar.setWidget(left);sidebar.setMaximumWidth(360);sidebar.setMinimumWidth(270)
        split.addWidget(sidebar)
        centre=QWidget();cl=QVBoxLayout(centre)
        self.turn=QLabel('Confirmed board • waiting for Chess Brain');self.turn.setObjectName('turn');cl.addWidget(self.turn)
        self.board=ChessBoard();cl.addWidget(self.board,1)
        row=QHBoxLayout();self.uci=QLineEdit();self.uci.setPlaceholderText('UCI move, e.g. e2e4');self.uci.setObjectName('uci_input')
        self.promotion=QComboBox()
        for label,key in [('Queen','q'),('Rook','r'),('Bishop','b'),('Knight','n')]:self.promotion.addItem(label,key)
        self.submit=QPushButton('Submit move');self.submit.setObjectName('submit_move');row.addWidget(self.uci,1);row.addWidget(self.promotion);row.addWidget(self.submit);cl.addLayout(row)
        self.submit.clicked.connect(lambda:self.move_requested.emit(self.uci.text().strip().lower()));self.uci.returnPressed.connect(lambda:self.move_requested.emit(self.uci.text().strip().lower()))
        self.plan_result=QLabel('Planning: —');self.execution_result=QLabel('Execution / confirmation: —');self.plan_result.setWordWrap(True);self.execution_result.setWordWrap(True);cl.addWidget(self.plan_result);cl.addWidget(self.execution_result)
        split.addWidget(centre)
        right=QWidget();rl=QVBoxLayout(right);right.setMaximumWidth(360)
        self.tabs=QTabWidget();rl.addWidget(self.tabs)
        tests=QWidget();tl=QVBoxLayout(tests);self.arm=QComboBox();self.arm.addItems(['robot1','robot2']);self.arm_label=QLabel('Arm for tests');tl.addWidget(self.arm_label);tl.addWidget(self.arm)
        row=QHBoxLayout();self.square=QComboBox();self.square.addItems([f+r for f in 'abcdefgh' for r in '12345678']);self.square.setCurrentText('e4');row.addWidget(self.square);tl.addLayout(row)
        self.square_buttons={}
        for label,key in [('Plan approach (no movement)','plan'),('Execute approach on mock controllers','execute'),('Empty-square probe + retreat','probe'),('Home / park selected arm','home')]:
            button=QPushButton(label);self.square_buttons[key]=button;button.clicked.connect(lambda checked=False,k=key:self.square_requested.emit(k));tl.addWidget(button)
        self.joint_box=QGroupBox('Model joints • waiting for description');self.joint_layout=QVBoxLayout(self.joint_box);self.joint_controls={};tl.addWidget(self.joint_box)
        self.apply_joints=QPushButton('Apply positions to Gazebo');self.apply_joints.clicked.connect(self.apply_joints_requested);tl.addWidget(self.apply_joints)
        self.operator=QPushButton('Confirm promotion replacement');self.operator.clicked.connect(self.promotion_requested);tl.addWidget(self.operator)
        self.fail_gripper=QPushButton('Fail next mock gripper operation');self.fail_gripper.clicked.connect(self.failure_requested);tl.addWidget(self.fail_gripper)
        self.hold_vision=QCheckBox('Hold fresh vision after robot ack');self.hold_vision.toggled.connect(self.vision_hold_requested);tl.addWidget(self.hold_vision)
        self.restart=QPushButton('Restart / recover simulated mode');self.restart.clicked.connect(self.restart_requested);tl.addWidget(self.restart)
        row=QHBoxLayout();self.sync=QPushButton('Request sync');self.state_button=QPushButton('Refresh state');row.addWidget(self.sync);row.addWidget(self.state_button);tl.addLayout(row)
        self.sync.clicked.connect(self.sync_requested);self.state_button.clicked.connect(self.state_requested)
        tl.addStretch();scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(tests);self.tabs.addTab(scroll,'Controls')
        self.history=QListWidget();self.tabs.addTab(self.history,'Confirmed moves')
        self.log=QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(1500);self.tabs.addTab(self.log,'Logs / errors')
        split.addWidget(right);split.setSizes([300,600,320])
        self.setStyleSheet('''QWidget { background:#14202d;color:#e1e9ef;font-size:13px; }
        QMainWindow {background:#14202d;} QLabel#heading {font-size:20px;font-weight:700;padding:8px;}
        QLabel#status {background:#203547;padding:10px;border-radius:6px;}
        QLabel#turn {font-size:16px;font-weight:600;}
        QPushButton {background:#294353;border:1px solid #4c6776;border-radius:5px;padding:8px;}
        QPushButton:hover {background:#365b6b;} QPushButton:disabled {color:#71808d;background:#1b2b38;}
        QPushButton#start_mode,QPushButton#submit_move {background:#24786d;} QPushButton#motion_stop {background:#943e40;}
        QLineEdit,QComboBox,QPlainTextEdit,QListWidget,QDoubleSpinBox {background:#1d3040;border:1px solid #425a69;border-radius:4px;padding:5px;}
        QGroupBox {border:1px solid #425a69;border-radius:4px;margin-top:9px;padding-top:12px;}
        QTabBar::tab {padding:8px;background:#203547;} QTabBar::tab:selected {background:#365b6b;}
        ''')
    def pick_profile(self):
        path,_=QFileDialog.getOpenFileName(self,'Select robot geometry YAML',self.profile.text(),'YAML (*.yaml *.yml)')
        if path:self.profile.setText(path)
    def closeEvent(self,event):
        if self.allow_close:event.accept()
        else:event.ignore();self.close_requested.emit()
    def show_joints(self,limits,arm,positions,enabled):
        while self.joint_layout.count():
            item=self.joint_layout.takeAt(0)
            if item.widget():item.widget().deleteLater()
        self.joint_controls={};self.joint_box.setTitle(arm+' • model limits / units')
        for name,lim in sorted(limits.items()):
            if not name.startswith(arm+'_'):continue
            label=QLabel(f"{name.split('_',1)[1]} ({lim['unit']})  [{lim['min']:.4f}, {lim['max']:.4f}]")
            self.joint_layout.addWidget(label)
            holder=QWidget();row=QHBoxLayout(holder);row.setContentsMargins(0,0,0,0)
            spin=QDoubleSpinBox();spin.setDecimals(5);spin.setRange(lim['min'],lim['max']);spin.setSingleStep(.001 if lim['unit']=='m' else .01)
            value=positions.get(name,(lim['min']+lim['max'])/2);spin.setValue(max(lim['min'],min(lim['max'],value)))
            slider=QSlider(Qt.Horizontal);slider.setRange(0,10000);slider.setValue(round((spin.value()-lim['min'])/(lim['max']-lim['min'])*10000));row.addWidget(slider,1);row.addWidget(spin);self.joint_layout.addWidget(holder)
            def slide(value,box=spin,lo=lim['min'],hi=lim['max']):box.setValue(lo+(hi-lo)*value/10000)
            def changed(value,s=slider,lo=lim['min'],hi=lim['max'],key=name):
                s.blockSignals(True);s.setValue(round((value-lo)/(hi-lo)*10000));s.blockSignals(False);self.joint_requested.emit(key,value)
            slider.valueChanged.connect(slide);spin.valueChanged.connect(changed)
            spin.setEnabled(enabled);slider.setEnabled(enabled);self.joint_controls[name]=(spin,slider,label)
