require 'torch'
require 'lfs'
require 'util.misc'

params = {}
params.num_classes = 41
local allClasses, allClassesNames = getClassInfo()
classes, classes_names = allClasses, allClassesNames
-- print(classes)




-- local methodNames = {'rbo','cnn_1','cnn_41','hha','hha_v2'}
-- local methodNames = {'apc_hha_lowres_0629_train_ep10','apc_hha_lowres_0629_train_ep20','apc_hha_lowres_0629_train_ep25'}
local methodNames = {'apc_hha_lowres_0629_train_001',
		    'apc_hha_lowres_0629_train_002',
		    'apc_hha_lowres_0629_train_003',
		    'apc_hha_lowres_0629_train_004',
		    'apc_hha_lowres_0629_train_005',
		    'apc_hha_lowres_0629_train_006',
		    'apc_hha_lowres_0629_train_007',
		    'apc_hha_lowres_0629_train_008',
		    'apc_hha_lowres_0629_train_009',
		    'apc_hha_lowres_0629_train_010'
}

local methodNames = {'apc_hha_lowres_0629_train_011_ep10',
		    'apc_hha_lowres_0629_train_011_ep20',
		    'apc_hha_lowres_0629_train_011_ep30',
		    'apc_hha_lowres_0629_train_011_ep30',
		    'apc_hha_lowres_0629_train_011_ep40',
		    'apc_hha_lowres_0629_train_011_ep50',
		    'apc_hha_lowres_0629_train_011_ep60',
		    'apc_hha_lowres_0629_train_011_ep70',
		    'apc_hha_lowres_0629_train_011_ep80',
		    'apc_hha_lowres_0629_train_011_ep90'
}
local methodNames = {'0805_tote_nodc_0',
                     '0805_tote_nodc',
                      '0805_tote_dc1',
                      '0805_tote'
}


-- local methodNames = {'apc_hha_lowres_0629_train_001_ep10',
--   'apc_hha_lowres_0629_train_001_ep20',
--   'apc_hha_lowres_0629_train_001_ep30',
--   'apc_hha_lowres_0629_train_001_ep40',
--   'apc_hha_lowres_0629_train_001',
-- }
  
for k,v in pairs(methodNames) do print(k,v) end		    
local allRes = torch.zeros(#methodNames, 41, 3)

for mn, methodName in pairs(methodNames) do
  
--   allRes[methodName] = torch.zeros(41,3) -- precision, recall, F1

  local files = {}
  for line in io.lines('../predictions/eval_'..methodName..'.txt') do 
    files[#files + 1] = line
    local className, p, r, f1 = 
      string.match(line,'([%a+%d+_]+)%s*Precision : ([%d.]*)%s*Recall : ([%d.]*)%s*F1%s*: ([%d.]*)')
    local classID = classNameToID(className, allClassesNames)
    allRes[mn][classID] = torch.Tensor({p,r,f1})
  end
--   print(allRes[methodName])
  -- remove box class from evaluation
  allRes[mn][2] = 0
end

-- HEADLINE
prLine = string.format('%-35s','Class')
for mn, methodName in pairs(methodNames) do
--   prLine = prLine .. string.format('%10s',methodName:sub(5,-1))
  prLine = prLine .. string.format('%10d',mn)
end
print(prLine)
print('---------------------------------------------------------------------------------')

for k,v in pairs(allClassesNames) do
  local prLine = string.format('%-35s',v)
  local mv, mi = torch.max(allRes[{{},{k},{3}}]:reshape(#methodNames),1)
  for mn, methodName in pairs(methodNames) do    
    if mv[1] == allRes[mn][k][3] then 
      prLine = prLine .. string.format('%4s%6.4f','*',allRes[mn][k][3])
    else
      prLine = prLine .. string.format('%10.4f',allRes[mn][k][3])
    end
    
  end
  print(prLine)
end

-- footer
print('---------------------------------------------------------------------------------')
prLine = string.format('%-35s','Mean')
for mn, methodName in pairs(methodNames) do
  prLine = prLine .. string.format('%10.4f',allRes[mn][{{},{3}}]:mean())
end
print(prLine..'\n')
