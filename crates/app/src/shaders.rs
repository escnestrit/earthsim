//! GLSL 3.30 core shaders, converted from the WebGL2 prototype (precision qualifiers dropped).
//! Material ids (vMat): 0 ground 1 sidewalk 2 asphalt 3 marking 4 wall 5 roof 6 paint 7 glass 8 tire 9 headlight 10 foliage 11 trunk 12 taillight 13 rim.

pub const MAIN_VS: &str = r####"#version 330 core
layout(location=0) in vec3 aPos;layout(location=1) in vec3 aN;layout(location=2) in vec2 aUV;layout(location=3) in float aMat;
uniform mat4 uVP,uModel;out vec3 vW;out vec3 vN;out vec2 vUV;flat out float vMat;
void main(){vec4 w=uModel*vec4(aPos,1.);vW=w.xyz;vN=mat3(uModel)*aN;vUV=aUV;vMat=aMat;gl_Position=uVP*w;}
"####;

pub const SHADOW_VS: &str = r####"#version 330 core
layout(location=0) in vec3 aPos;uniform mat4 uLightVP,uModel;
void main(){gl_Position=uLightVP*uModel*vec4(aPos,1.);}
"####;

pub const SHADOW_FS: &str = r####"#version 330 core
void main(){}
"####;

pub const MAIN_FS: &str = r####"#version 330 core

in vec3 vW;in vec3 vN;in vec2 vUV;flat in float vMat;
uniform vec3 uCam,uLightDir,uSunCol,uSkyAmb,uGroundAmb,uHorizon,uZenith,uPaint,uHeadPos,uHeadDir;
uniform float uSunI,uDay,uShadowOn,uFogD,uHeadOn,uBrake;
uniform mat4 uLightVP;uniform sampler2DShadow uShadow;
out vec4 o;

uniform float uExposure;
float hash(vec2 p){p=fract(p*vec2(123.34,456.21));p+=dot(p,p+45.32);return fract(p.x*p.y);}
float vnoise(vec2 p){vec2 i=floor(p),f=fract(p);f=f*f*(3.-2.*f);return mix(mix(hash(i),hash(i+vec2(1,0)),f.x),mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x),f.y);}
vec3 lin(vec3 c){return pow(c,vec3(2.2));}
vec3 tone(vec3 x){x*=uExposure;x=clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.,1.);return pow(x,vec3(1./2.2));}
float box1(vec2 p,vec2 lo,vec2 hi,vec2 aa){vec2 a=smoothstep(lo-aa,lo+aa,p)*(1.-smoothstep(hi-aa,hi+aa,p));return a.x*a.y;}
float shadowAt(vec3 wp,vec3 n){
 vec3 p=wp+n*0.12;vec4 lp=uLightVP*vec4(p,1.);vec3 pc=lp.xyz/lp.w*0.5+0.5;
 if(pc.x<0.||pc.x>1.||pc.y<0.||pc.y>1.||pc.z>1.)return 1.;
 float s=0.;for(int i=-1;i<=1;i++)for(int j=-1;j<=1;j++)s+=texture(uShadow,vec3(pc.xy+vec2(float(i),float(j))/2048.,pc.z-0.0008));
 s/=9.;vec2 e=min(pc.xy,1.-pc.xy);float fade=smoothstep(0.0,0.08,min(e.x,e.y));return mix(1.,s,fade);}
void main(){
 vec3 N=normalize(vN);vec3 V=normalize(uCam-vW);vec3 L=uLightDir;
 int m=int(vMat+0.5);vec3 alb=vec3(.5),emis=vec3(0);float specAmt=0.,reflAmt=0.;
 vec3 R=reflect(-V,N);vec3 skyR=mix(uHorizon,uZenith,clamp(R.z,0.,1.));
 float night=1.-uDay;
 if(m==0){float n1=vnoise(vW.xy*.15),n2=vnoise(vW.xy*1.3),n3=vnoise(vW.xy*6.);alb=lin(mix(vec3(.30,.40,.18),vec3(.42,.44,.24),n1)*(.85+.3*n2)*(.92+.16*n3));}
 else if(m==1){vec2 g=abs(fract(vW.xy/.7)-.5);float ln=smoothstep(.46,.5,max(g.x,g.y));alb=lin(vec3(.55,.54,.52)*(.9+.2*vnoise(vW.xy*2.))*(1.-.2*ln));}
 else if(m==2){float n=vnoise(vW.xy*.5)*.4+vnoise(vW.xy*3.)*.3+vnoise(vW.xy*17.)*.3;alb=lin(vec3(.27,.27,.28)*(.88+.24*n));}
 else if(m==3){alb=lin(vec3(.88,.88,.82));}
 else if(m==4){
  float seed=floor(vUV.x*1024.+.5)/1024.;   // quantize: interpolation error would make the per-building hash flicker
  vec3 pal[5]=vec3[5](vec3(.80,.76,.68),vec3(.70,.71,.72),vec3(.62,.36,.29),vec3(.84,.76,.54),vec3(.66,.72,.78));
  vec3 base=pal[int(seed*5.)%5];
  float along=dot(vW.xy,vec2(-N.y,N.x));float cw=2.8+fract(seed*13.)*1.4;
  float cu=fract(along/cw),fl=vW.z/3.,cv=fract(fl),fi=floor(fl);
  vec2 aa=fwidth(vec2(along/cw,fl))*.8+1e-4;
  vec3 wall=base*(.88+.2*vnoise(vec2(along*.8,vW.z*.8)));
  wall*=1.-.10*smoothstep(.03,.0,cv);
  wall*=mix(.72,1.,smoothstep(0.,5.,vW.z));
  float win=0.,ring=0.;
  if(fi<1.){win=box1(vec2(cu,cv),vec2(.06,.12),vec2(.94,.72),aa);}
  else{win=box1(vec2(cu,cv),vec2(.24,.28),vec2(.76,.80),aa);ring=box1(vec2(cu,cv),vec2(.19,.23),vec2(.81,.85),aa)-win;}
  float fres=pow(1.-max(dot(N,V),0.),3.)*.7+.12;
  vec3 glass=mix(lin(vec3(.07,.09,.12)),skyR*.55,fres);
  float cellId=floor(along/cw)+floor(seed*1024.);float litR=hash(vec2(cellId,fi+floor(seed*1024.)*.37));
  if(litR>.60&&fi>=1.){emis=vec3(1.,.78,.45)*1.6*win*night;glass=mix(glass,vec3(.5,.4,.25),night*.8);}
  if(fi<1.&&litR>.35){emis=vec3(1.,.9,.7)*1.8*win*night;}
  alb=mix(wall,glass,win);alb=mix(alb,lin(vec3(.85,.85,.82)),clamp(ring,0.,1.)*.8);
  specAmt=win*.4;reflAmt=win*fres*.5;
 }
 else if(m==5){alb=lin(mix(vec3(.32,.30,.28),vec3(.42,.22,.17),fract(vUV.x*7.))*(.85+.3*vnoise(vW.xy*.6)));}
 else if(m==6){alb=lin(uPaint);specAmt=1.0;reflAmt=.07;}
 else if(m==7){alb=lin(vec3(.03,.04,.05));specAmt=1.5;reflAmt=.8;}
 else if(m==8){alb=lin(vec3(.05));}
 else if(m==13){alb=lin(vec3(.55));specAmt=.8;}
 else if(m==9){alb=lin(vec3(.8));emis=vec3(1.,.95,.8)*4.*uHeadOn;}
 else if(m==12){alb=lin(vec3(.4,.02,.02));emis=vec3(1.,.04,.02)*(.6+uBrake*7.)*(.3+.7*night+uBrake);}
 else if(m==10){float sd=vUV.x;alb=lin(mix(vec3(.12,.30,.08),vec3(.27,.43,.12),fract(sd*9.))*(.8+.4*vnoise(vW.xy*3.+vW.z)));}
 else if(m==11){alb=lin(vec3(.25,.18,.12));}
 float sh=1.;if(uShadowOn>.5)sh=shadowAt(vW,N);
 float ndl=max(dot(N,L),0.);
 vec3 amb=mix(uGroundAmb,uSkyAmb,N.z*.5+.5);
 vec3 hd=vW-uHeadPos;float hdist=length(hd);vec3 hdn=hd/max(hdist,.01);
 float spot=smoothstep(.84,.95,dot(hdn,uHeadDir))*uHeadOn/(1.+.003*hdist*hdist)*clamp(1.-hdist/90.,0.,1.);
 float hn=.25+.75*max(dot(N,-hdn),0.);
 vec3 light=uSunCol*uSunI*ndl*sh+amb+vec3(1.,.93,.78)*30.*spot*hn;
 vec3 col=alb*light+emis;
 vec3 H=normalize(L+V);col+=uSunCol*uSunI*sh*pow(max(dot(N,H),0.),90.)*specAmt*step(.001,ndl);
 col+=skyR*reflAmt*(.4+.6*uDay);
 float fd=length(vW-uCam);col=mix(col,uHorizon,1.-exp(-fd*uFogD));
 o=vec4(tone(col),1.);}
"####;

pub const SKY_VS: &str = r####"#version 330 core
out vec2 vP;void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));vP=p*2.-1.;gl_Position=vec4(vP,.9999,1.);}
"####;

pub const SKY_FS: &str = r####"#version 330 core
in vec2 vP;out vec4 o;
uniform vec3 uFwd,uRight,uUp,uZenith,uHorizon,uSunDirS,uSunColS;uniform vec2 uTan;uniform float uDay;

uniform float uExposure;
float hash(vec2 p){p=fract(p*vec2(123.34,456.21));p+=dot(p,p+45.32);return fract(p.x*p.y);}
float vnoise(vec2 p){vec2 i=floor(p),f=fract(p);f=f*f*(3.-2.*f);return mix(mix(hash(i),hash(i+vec2(1,0)),f.x),mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x),f.y);}
vec3 lin(vec3 c){return pow(c,vec3(2.2));}
vec3 tone(vec3 x){x*=uExposure;x=clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.,1.);return pow(x,vec3(1./2.2));}
void main(){vec3 d=normalize(uFwd+vP.x*uTan.x*uRight+vP.y*uTan.y*uUp);
 float t=clamp(d.z,0.,1.);vec3 col=mix(uHorizon,uZenith,pow(t,.5));
 float sd=max(dot(d,uSunDirS),0.);float up=smoothstep(-.06,.12,uSunDirS.z);
 col+=uSunColS*(pow(sd,1800.)*40.+pow(sd,12.)*.25+pow(sd,3.)*.05)*up;
 float st=step(.9988,hash(floor(d.xy/(d.z+1.3)*260.)))*(1.-uDay)*smoothstep(0.,.15,d.z);col+=vec3(.7,.75,.9)*st;
 o=vec4(tone(col),1.);}
"####;

pub const HUD_VS: &str = r####"#version 330 core
layout(location=0) in vec2 aPos;layout(location=1) in vec2 aUV;
uniform vec2 uScreen;out vec2 vUV;
void main(){vec2 p=vec2(aPos.x/uScreen.x*2.-1.,1.-aPos.y/uScreen.y*2.);vUV=aUV;gl_Position=vec4(p,0.,1.);}
"####;

pub const HUD_FS: &str = r####"#version 330 core
in vec2 vUV;uniform sampler2D uTex;uniform vec4 uColor;uniform float uSolid;out vec4 o;
void main(){float a=uSolid>.5?1.:texture(uTex,vUV).r;o=vec4(uColor.rgb,uColor.a*a);}
"####;
