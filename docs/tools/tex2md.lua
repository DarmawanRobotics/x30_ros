-- pandoc filter of tex2md.py: the LaTeX documents' constructs as VitePress Markdown.
--   center, small: unwrapped
--   notebox, tipbox, warnbox, dangerbox: ::: info|tip|warning|danger <title> containers
--   headings: numbered like the PDF (-M chapter=N), with their label as the anchor {#...}
--   images: the figure's SVG of the site (docs/site/public/figures/<name>.svg)
--   description lists: bullets with the term in bold (VitePress has no definition lists)

local chapter = ''
local section, subsection = 0, 0

local BOXES = { notebox = 'info', tipbox = 'tip', warnbox = 'warning', dangerbox = 'danger' }

local function anchor(label)
  return (label:lower():gsub('[^a-z0-9-]+', '-'):gsub('^-+', ''):gsub('-+$', ''))
end

local function markdown(inlines)
  local doc = pandoc.Pandoc({ pandoc.Plain(inlines) })
  local text = pandoc.write(doc, 'gfm+tex_math_dollars', { wrap_text = 'none' })
  return (text:gsub('\n+$', ''))
end

function Meta(meta)
  if meta.chapter then
    chapter = pandoc.utils.stringify(meta.chapter)
  end
end

function Div(div)
  local kind = BOXES[div.classes[1]]
  if kind then
    local title = ''
    local body = {}
    for _, block in ipairs(div.content) do
      if block.t == 'Div' and block.classes[1] == 'boxtitle' then
        title = pandoc.utils.stringify(block)
      else
        table.insert(body, block)
      end
    end
    local out = { pandoc.RawBlock('markdown', '::: ' .. kind .. ' ' .. title) }
    for _, block in ipairs(body) do
      table.insert(out, block)
    end
    table.insert(out, pandoc.RawBlock('markdown', ':::'))
    return out
  end
  if div.classes[1] == 'center' or div.classes[1] == 'small' then
    return div.content
  end
end

function Header(h)
  local text = markdown(h.content)
  if h.level == 1 then
    return pandoc.RawBlock('markdown', '# ' .. text)
  end
  local number = ''
  if chapter ~= '' then
    if h.level == 2 then
      section, subsection = section + 1, 0
      number = chapter .. '.' .. section .. ' '
    elseif h.level == 3 then
      subsection = subsection + 1
      number = chapter .. '.' .. section .. '.' .. subsection .. ' '
    end
  end
  local id = h.identifier ~= '' and (' {#' .. anchor(h.identifier) .. '}') or ''
  return pandoc.RawBlock('markdown', string.rep('#', h.level) .. ' ' .. number .. text .. id)
end

-- display math as its own block: markdown-it-mathjax3 renders $$ on lines of their own
local function blank(x)
  return x.t == 'Space' or x.t == 'SoftBreak' or x.t == 'LineBreak'
end

local function split(block)
  local out, current, changed = {}, {}, false
  for _, x in ipairs(block.content) do
    if x.t == 'Math' and x.mathtype == 'DisplayMath' then
      changed = true
      while #current > 0 and blank(current[#current]) do
        table.remove(current)
      end
      if #current > 0 then
        table.insert(out, pandoc.Para(current))
      end
      local tex = x.text:gsub('^%s+', ''):gsub('%s+$', '')
      table.insert(out, pandoc.RawBlock('markdown', '$$\n' .. tex .. '\n$$'))
      current = {}
    elseif not (#current == 0 and blank(x)) then
      table.insert(current, x)
    end
  end
  if not changed then
    return nil
  end
  if #current > 0 then
    table.insert(out, pandoc.Para(current))
  end
  return out
end

function Para(p)
  return split(p)
end

-- inline math on one line: a line break inside $...$ ends it for markdown-it
function Math(m)
  if m.mathtype == 'InlineMath' then
    m.text = m.text:gsub('%s*\n%s*', ' ')
    return m
  end
end

function Image(img)
  if not img.src:match('^%a+:') and not img.src:match('%.%a+$') then
    img.src = '../../public/figures/' .. img.src .. '.svg'
  end
  img.attributes = {}
  img.caption = {}
  return img
end

function DefinitionList(dl)
  local items = {}
  for _, entry in ipairs(dl.content) do
    local term, definitions = entry[1], entry[2]
    local inlines = { pandoc.Strong(term) }
    local rest = {}
    for i, blocks in ipairs(definitions) do
      for j, block in ipairs(blocks) do
        if i == 1 and j == 1 and (block.t == 'Para' or block.t == 'Plain') then
          table.insert(inlines, pandoc.Space())
          for _, x in ipairs(block.content) do
            table.insert(inlines, x)
          end
        else
          table.insert(rest, block)
        end
      end
    end
    local item = { pandoc.Plain(inlines) }
    for _, block in ipairs(rest) do
      table.insert(item, block)
    end
    table.insert(items, item)
  end
  return pandoc.BulletList(items)
end

return {
  { Meta = Meta },
  { Div = Div, Header = Header, Image = Image, DefinitionList = DefinitionList, Para = Para, Math = Math },
}
